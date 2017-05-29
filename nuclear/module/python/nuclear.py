import inspect
import message
import re
import os
from textwrap import dedent

def indent(str, len=4):
    return '\n'.join([(' ' * len) + l for l in str.splitlines()])

class DSLWord(object):
    pass

# Single type keywords

class SingleTypeDSLWord(DSLWord):

    def __init__(self, t):
        self._name = self.__class__.__name__
        self._t = t
        self._include_paths = [ t.include_path() ]

    def template_args(self):
        return "{}<::{}::{}>".format(self._name, self._t.__module__.replace('.', '::'), self._t.__name__.replace('.', '::'))

    def input_types(self):
        return ["const {}::{}&".format(self._t.__module__.replace('.', '::'), self._t.__name__.replace('.', '::'))]

    def include_paths(self):
        return self._include_paths

class Trigger(SingleTypeDSLWord): pass
class With(SingleTypeDSLWord): pass
class Sync(SingleTypeDSLWord): pass

# No arg Keywords

class NoArgsDSLWord(DSLWord):
    def __init__(self):
        self._name = self.__class__.__name__

    def template_args(self):
        return self._name

class Always(NoArgsDSLWord): pass
class MainThread(NoArgsDSLWord): pass
class Single(NoArgsDSLWord): pass
class Startup(NoArgsDSLWord): pass
class Shutdown(NoArgsDSLWord): pass

# DSL modifiers

class Last(DSLWord):
    def __init__(self, dsl, count):
        self._name = self.__class__.__name__
        self._dsl = dsl
        self._count = count

    def template_args(self):
        return "{}<{}, {}>".format(self._name, self._count, self._dsl.template_args())

    def runtime_args(self):
        return self._dsl.runtime_args()

    def include_paths(self):
        return self._dsl.include_paths()

class Optional(DSLWord):
    def __init__(self, dsl):
        self._name = self.__class__.__name__
        self._dsl = dsl

    def template_args(self):
        return "{}<{}>".format(self._name, self._dsl.template_args())

    def runtime_args(self):
        return self._dsl.runtime_args()

    def include_paths(self):
        return self._dsl.include_paths()

# Weird types

class Buffer(DSLWord):
    def __init__(self, k):
        self._name = self.__class__.__name__
        self._k = k

    def template_args(self):
        return "{}<{}>".format(self._name, self._k)

class Every(DSLWord):
    def __init__(self, time):
        self._name = self.__class__.__name__
        self._time = time

    def template_args(self):
        ns = int(self._time * 1e9)
        return "{}<{}, std::chrono::nanoseconds>".format(self._name, ns)


class Priority(object):
    class REALTIME(DSLWord):
        def template_arg(self):
            return "Priority::REALTIME"

    class HIGH(DSLWord):
        def template_arg(self):
            return "Priority::HIGH"

    class NORMAL(DSLWord):
        def template_arg(self):
            return "Priority::NORMAL"

    class LOW(DSLWord):
        def template_arg(self):
            return "Priority::LOW"

    class IDLE(DSLWord):
        def template_arg(self):
            return "Priority::IDLE"


# Type that holds a DSL function

class DSLCallback(DSLWord):

    def __init__(self, func, *dsl):
        self.func = func

        self._t_args = ", ".join(w.template_args() for w in dsl if hasattr(w, 'template_args') and w.template_args())

        self._r_args = ", ".join(w.runtime_args()  for w in dsl if hasattr(w, 'runtime_args')  and w.runtime_args())

        paths = [w.include_paths() for w in dsl if hasattr(w, 'include_paths') and w.include_paths()]
        self._include_paths = [b for a in paths for b in a]

        inputs = [w.input_types() for w in dsl if hasattr(w, 'input_types') and w.input_types()]
        self._i_args = [b for a in inputs for b in a]

    def template_args(self):
        return self._t_args

    def runtime_args(self):
        return self._r_args

    def input_types(self):
        return self._i_args

    def include_paths(self):
        return self._include_paths

    def function(self):
        return self.func

# Decorator for creating instance variables/setting up reactor
def Reactor(reactor):

    # Attach an emit method to the class
    setattr(reactor, 'emit', lambda self, msg: msg._emit(self._reactor_ptr))

    try:
        # If we can import this we are running in nuclear, so run
        import nuclear_reactor

        # Bind an instance of this reactor into nuclear
        instance = reactor()
        setattr(instance, '_reactor_ptr', nuclear_reactor.bind_self(instance))

        # Go through all of our DSL callbacks to bind them
        reactions = inspect.getmembers(reactor, predicate=lambda x: isinstance(x, DSLCallback))
        for reaction in reactions:
            func_name = 'bind_{}'.format(re.sub(r'(?:\W|^(?=\d))+', '_', reaction[1].template_args()))
            getattr(nuclear_reactor, func_name)(reaction[1].function())

    # If we don't have nuclear_reactor defined, we generate the c++
    except ImportError:
        # Get the filename of the reactor (presumably the person who included us)
        reactor_abs_path = os.path.abspath(inspect.stack()[1].filename)

        # Get the module base directory or default to our current dir
        module_dir = os.path.normpath(os.getenv('NUCLEAR_MODULE_DIR', os.getcwd()))

        # Get the path for the reactor code
        reactor_path = os.path.relpath(reactor_abs_path, module_dir)
        # Strip to the src directory
        reactor_dir = os.path.dirname(reactor_path)
        # Strip to the name of the module
        reactor_namespace = os.path.join('module', os.path.dirname(os.path.dirname(reactor_dir)))

        # Get the reactor name (the name of the class)
        reactor_name = reactor.__name__

        # Get our reactions
        reactions = inspect.getmembers(reactor, predicate=lambda x: isinstance(x, DSLCallback))

        binder_impl = dedent("""\
            // Binding function for the dsl on<{dsl}>
            m.def("bind_{func_name}", [this] (pybind11::function fn) {{

                on<{dsl}>().then([this, fn] ({input_args}) {{

                    // Create our thread state for this thread if it doesn't exist
                    if (!thread_state) {{
                        thread_state = PyThreadState_New(interpreter);
                    }}

                    // Load our thread state
                    PyEval_RestoreThread(thread_state);

                    // Run the python function
                    try {{
                        fn({input_vars});
                        PyEval_SaveThread();
                    }}
                    catch(...) {{
                        // Finished with python
                        PyEval_SaveThread();
                        std::rethrow_exception(std::current_exception());
                    }}
                }});
            }});""")

        binders = set()
        includes = set()

        # Loop through our reactions and add handler functions for them
        for reaction in reactions:
            func_name = re.sub(r'(?:\W|^(?=\d))+', '_', reaction[1].template_args())

            input_types = reaction[1].input_types()
            input_vars = ['var{}'.format(i) for i in range(len(input_types))]
            input_args = ['{} {}'.format(arg, var) for arg, var in zip(input_types, input_vars)]

            binders.add(binder_impl.format(func_name=func_name,
                                           dsl=reaction[1].template_args(),
                                           input_args=', '.join(input_args),
                                           input_types=', '.join(input_types),
                                           input_vars=', '.join(['self'] + input_vars)))

            for include in reaction[1].include_paths():
                includes.add('#include "{}"'.format(include))

        macro_guard = "{}_H".format(reactor_name.upper())
        header_file = "{}.h".format(reactor_name)
        open_namespace  = '\n'.join('namespace {} {{'.format(n) for n in reactor_namespace.split(os.path.sep))
        close_namespace = '\n'.join('}}  // namespace {}'.format(n) for n in reactor_namespace.split(os.path.sep))

        header_template = dedent("""\
            #ifndef {macro_guard}
            #define {macro_guard}

            #include <nuclear>
            #include <pybind11/pybind11.h>
            #include <Python.h>

            {open_namespace}

                class {class_name} : public NUClear::Reactor {{
                public:
                    // Constructor
                    explicit {class_name}(std::unique_ptr<NUClear::Environment> environment);
                    {class_name}(const {class_name}&) = default;
                    {class_name}({class_name}&&) = default;
                    ~{class_name}() = default;
                    {class_name}& operator=(const {class_name}&) = default;
                    {class_name}& operator=({class_name}&&) = default;

                private:
                    // The subinterpreter for this module
                    PyInterpreterState* interpreter;

                    // The self object for this module
                    pybind11::object self;

                    // The thread state for this thread/interpreter combination
                    static thread_local PyThreadState* thread_state;
                }};
            {close_namespace}

            #endif  // {macro_guard}
            """)

        with open(os.getcwd() + os.sep + reactor_name + '.h', 'w') as f:
            f.write(header_template.format(class_name=reactor_name,
                                           macro_guard=macro_guard,
                                           open_namespace=open_namespace,
                                           close_namespace=close_namespace))

        cpp_template = dedent("""\
            #include "{header_file}"

            #include <pybind11/functional.h>

            {includes}

            // Declare our message init function (comes from the messages code)
            extern "C" {{
                PyObject* PyInit_message();
            }}

            {open_namespace}

                thread_local PyThreadState* {class_name}::thread_state = nullptr;

                {class_name}::{class_name}(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)), interpreter(nullptr), self() {{
                    // If python hasn't been used in another module yet
                    if (!Py_IsInitialized()) {{
                        // Add our message to our initilsation
                        PyImport_AppendInittab("message", &PyInit_message);

                        // Initialise without signal handlers
                        Py_InitializeEx(0);

                        // Initialise using threads in python and then release the GIL
                        PyEval_InitThreads();
                        PyEval_ReleaseLock();
                    }}

                    // Acquire the gil
                    PyEval_AcquireLock();

                    // This sets the threadstate/interpreter combination for
                    // the thread that creates this interperter
                    thread_state = Py_NewInterpreter();

                    // Store a pointer to our newly created interpreter
                    interpreter = thread_state->interp;

                    // Create a module object that holds our binding functions
                    pybind11::module m("nuclear_reactor", "Binding functions for the current nuclear reactor");

                    // Create a function that binds the self object for passing into callbacks
                    m.def("bind_self", [this] (pybind11::object obj) {{
                        // Store the provided object
                        self = obj;

                        // Return the reactor as an opaque object (for emitting)
                        return pybind11::capsule(this);
                    }});

            {binders}

                    // Take our created module and add it to this subinterpreters imports
                    PyImport_AddModule("nuclear_reactor");
                    PyObject* sys_modules = PyImport_GetModuleDict();
                    PyDict_SetItemString(sys_modules, "nuclear_reactor", m.ptr());

                    // Setup our search path so that it can find other files and nuclear.py
                    PyObject* sysPath = PySys_GetObject("path");

                    pybind11::str nuclear_path("{nuclear_directory}");
                    pybind11::str script_path("{reactor_directory}");

                    PyList_Append(sysPath, nuclear_path.ptr());
                    PyList_Append(sysPath, script_path.ptr());

                    // Now open up our main python file and run it to bind all the functions
                    PyRun_SimpleFile(fopen("{python_file}", "r"), "{python_file}");

                    // Release the GIL
                    PyEval_ReleaseLock();
                }}

            {close_namespace}
            """)

        with open(os.getcwd() + os.sep + reactor_name + '.cpp', 'w') as f:
            f.write(cpp_template.format(header_file=header_file,
                                        class_name=reactor_name,
                                        includes='\n'.join(includes),
                                        nuclear_directory=os.path.join('python', 'nuclear'),
                                        reactor_directory=os.path.join('python', reactor_dir),
                                        python_file=os.path.join('python', reactor_path),
                                        binders=indent('\n\n'.join(binders), 8),
                                        open_namespace=open_namespace,
                                        close_namespace=close_namespace))

    return reactor

# Our on function that creates the binding
def on(*args):

    def decorator(func):

        return DSLCallback(func, *args)

    return decorator

# TYPES THAT I DON'T KNOW IF I WANT

# IO? probably has no analouge in python
# TCP
# UDP

# Network
