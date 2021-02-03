`./b target <PLATFORM>`
 - Calls dockerise.build_platform()
 - Pulls the latest image for given platform from docker hub
 - Also builds a image locally using docker/Dockerfile?

`./b configure`
 - Does something

`./b configure -i`
 - Run interactivly and choose what roles to build

`./b build`
 -





cd /home/nubots/build && /usr/bin/ctest --force-new-ctest-process --output-on-failure --output-log /home/nubots/NUbots/TEST.log




 Errors while running CTest
FAILED: CMakeFiles/test.util
cd /home/nubots/build && /usr/bin/ctest --force-new-ctest-process
ninja: build stopped: subcommand failed.




Tests path: /home/nubots/build/Testing/Temporary



"Mounts": [
            {
                "Type": "bind",
                "Source": "/home/yoshi/Code/NUbots",
                "Destination": "/home/nubots/NUbots",
                "Mode": "",
                "RW": true,
                "Propagation": "rprivate"
            },
            {
                "Type": "volume",
                "Name": "nubots_generic_build",
                "Source": "/var/lib/docker/volumes/nubots_generic_build/_data",
                "Destination": "/home/nubots/build",
                "Driver": "local",
                "Mode": "z",
                "RW": true,
                "Propagation": ""
            }
        ],


[nubots@docker build]$ /usr/bin/ctest -help
Usage

  ctest [options]

Options
  -C <cfg>, --build-config <cfg>
                               = Choose configuration to test.
  --progress                   = Enable short progress output from tests.
  -V,--verbose                 = Enable verbose output from tests.
  -VV,--extra-verbose          = Enable more verbose output from tests.
  --debug                      = Displaying more verbose internals of CTest.
  --output-on-failure          = Output anything outputted by the test
                                 program if the test should fail.
  --stop-on-failure            = Stop running the tests after one has failed.
  --test-output-size-passed <size>
                               = Limit the output for passed tests to <size>
                                 bytes
  --test-output-size-failed <size>
                               = Limit the output for failed tests to <size>
                                 bytes
  -F                           = Enable failover.
  -j <jobs>, --parallel <jobs> = Run the tests in parallel using the given
                                 number of jobs.
  -Q,--quiet                   = Make ctest quiet.
  -O <file>, --output-log <file>
                               = Output to log file
  -N,--show-only[=format]      = Disable actual execution of tests.  The
                                 optional 'format' defines the format of the
                                 test information and can be 'human' for the
                                 current text format or 'json-v1' for json
                                 format.  Defaults to 'human'.
  -L <regex>, --label-regex <regex>
                               = Run tests with labels matching regular
                                 expression.
  -R <regex>, --tests-regex <regex>
                               = Run tests matching regular expression.
  -E <regex>, --exclude-regex <regex>
                               = Exclude tests matching regular expression.
  -LE <regex>, --label-exclude <regex>
                               = Exclude tests with labels matching regular
                                 expression.
  -FA <regex>, --fixture-exclude-any <regex>
                               = Do not automatically add any tests for
                                 fixtures matching regular expression.
  -FS <regex>, --fixture-exclude-setup <regex>
                               = Do not automatically add setup tests for
                                 fixtures matching regular expression.
  -FC <regex>, --fixture-exclude-cleanup <regex>
                               = Do not automatically add cleanup tests for
                                 fixtures matching regular expression.
  -D <dashboard>, --dashboard <dashboard>
                               = Execute dashboard test
  -D <var>:<type>=<value>      = Define a variable for script mode
  -M <model>, --test-model <model>
                               = Sets the model for a dashboard
  -T <action>, --test-action <action>
                               = Sets the dashboard action to perform
  --group <group>              = Specify what build group on the dashboard
                                 you'd like to submit results to.
  -S <script>, --script <script>
                               = Execute a dashboard for a configuration
  -SP <script>, --script-new-process <script>
                               = Execute a dashboard for a configuration
  -A <file>, --add-notes <file>= Add a notes file with submission
  -I [Start,End,Stride,test#,test#|Test file], --tests-information
                               = Run a specific number of tests by number.
  -U, --union                  = Take the Union of -I and -R
  --rerun-failed               = Run only the tests that failed previously
  --repeat until-fail:<n>, --repeat-until-fail <n>
                               = Require each test to run <n> times without
                                 failing in order to pass
  --repeat until-pass:<n>      = Allow each test to run up to <n> times in
                                 order to pass
  --repeat after-timeout:<n>   = Allow each test to run up to <n> times if it
                                 times out
  --max-width <width>          = Set the max width for a test name to output
  --interactive-debug-mode [0|1]
                               = Set the interactive mode to 0 or 1.
  --resource-spec-file <file>  = Set the resource spec file to use.
  --no-label-summary           = Disable timing summary information for
                                 labels.
  --no-subproject-summary      = Disable timing summary information for
                                 subprojects.
  --build-and-test             = Configure, build and run a test.
  --build-target               = Specify a specific target to build.
  --build-nocmake              = Run the build without running cmake first.
  --build-run-dir              = Specify directory to run programs from.
  --build-two-config           = Run CMake twice
  --build-exe-dir              = Specify the directory for the executable.
  --build-generator            = Specify the generator to use.
  --build-generator-platform   = Specify the generator-specific platform.
  --build-generator-toolset    = Specify the generator-specific toolset.
  --build-project              = Specify the name of the project to build.
  --build-makeprogram          = Specify the make program to use.
  --build-noclean              = Skip the make clean step.
  --build-config-sample        = A sample executable to use to determine the
                                 configuration
  --build-options              = Add extra options to the build step.
  --test-command               = The test to run with the --build-and-test
                                 option.
  --test-timeout               = The time limit in seconds, internal use
                                 only.
  --test-load                  = CPU load threshold for starting new parallel
                                 tests.
  --tomorrow-tag               = Nightly or experimental starts with next day
                                 tag.
  --overwrite                  = Overwrite CTest configuration option.
  --extra-submit <file>[;<file>]
                               = Submit extra files to the dashboard.
  --force-new-ctest-process    = Run child CTest instances as new processes
  --schedule-random            = Use a random order for scheduling tests
  --submit-index               = Submit individual dashboard tests with
                                 specific index
  --timeout <seconds>          = Set the default test timeout.
  --stop-time <time>           = Set a time at which all tests should stop
                                 running.
  --http1.0                    = Submit using HTTP 1.0.
  --no-compress-output         = Do not compress test output when submitting.
  --print-labels               = Print all available test labels.
  --no-tests=<[error|ignore]>  = Regard no tests found either as 'error' or
                                 'ignore' it.
  --help,-help,-usage,-h,-H,/? = Print usage information and exit.
  --version,-version,/V [<f>]  = Print version number and exit.
  --help-full [<f>]            = Print all help manuals and exit.
  --help-manual <man> [<f>]    = Print one help manual and exit.
  --help-manual-list [<f>]     = List help manuals available and exit.
  --help-command <cmd> [<f>]   = Print help for one command and exit.
  --help-command-list [<f>]    = List commands with help available and exit.
  --help-commands [<f>]        = Print cmake-commands manual and exit.
  --help-module <mod> [<f>]    = Print help for one module and exit.
  --help-module-list [<f>]     = List modules with help available and exit.
  --help-modules [<f>]         = Print cmake-modules manual and exit.
  --help-policy <cmp> [<f>]    = Print help for one policy and exit.
  --help-policy-list [<f>]     = List policies with help available and exit.
  --help-policies [<f>]        = Print cmake-policies manual and exit.
  --help-property <prop> [<f>] = Print help for one property and exit.
  --help-property-list [<f>]   = List properties with help available and
                                 exit.
  --help-properties [<f>]      = Print cmake-properties manual and exit.
  --help-variable var [<f>]    = Print help for one variable and exit.
  --help-variable-list [<f>]   = List variables with help available and exit.
  --help-variables [<f>]       = Print cmake-variables manual and exit.
