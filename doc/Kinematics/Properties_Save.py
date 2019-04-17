#Author-
#Description-
import adsk.core, adsk.fusion, traceback, yaml, math
import numpy as np

def run(context):
    try:
        app = adsk.core.Application.get()

        if not app.activeDocument.name:
            app.userInterface.messageBox('No active Fusion product', 'No product')
            return

        # Get the root component of the active product.
        rootComp = app.activeDocument.name.rootComponent

        bodies = [body for body in rootComp.bRepBodies]

        # Iterate through all of the occurrences in the assembly.
        for occ in rootComp.allOccurrences:
            # Iterate over all of the bodies within the component.
            bodies += [x for x in occ.component.bRepBodies]

        data = {}

        # Iterate through all of the bodies in the occurrences.
        for body in bodies:
            physicalProperties = body.physicalProperties
            name = body.parentComponent.name.replace(' ', '_')

            # Get Bounding Box from physical properties
            bBoxMax = body.boundingBox.maxPoint
            bBoxMin = body.boundingBox.minPoint

            # Get the moment of inertia about the world coordinate system.
            (retVal, xx, yy, zz, xy, yz, xz) = physicalProperties.getXYZMomentsOfInertia()

            t_origin = np.asarray([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])

            # Get center of mass from physical properties
            CoM = physicalProperties.centerOfMass
            (x, y, z) = (CoM.x, CoM.y, CoM.z)

            d = np.asarray([[y**2 + z**2, -x*y, -x*z], [-x*y, x**2 + z**2, -y*z], [-x*z, -y*z, x**2 + y**2]])

            t_com = t_origin - physicalProperties.mass * d

            # kg.cm^2 to g.mm^2
            # t_mm = t_com * 1000 * 10 * 10

            # kg.cm^2 to m^2 (unit mass)
            t_com = (t_com / physicalProperties.mass) * 100 * 100

            data[name] = {
                'Area':     physicalProperties.area / 100 / 100,
                'Volume':   physicalProperties.volume / 100 / 100 / 100,
                'bBoxMin':  [bBoxMin.x / 100, bBoxMin.y / 100, bBoxMin.z / 100],
                'bBoxMax':  [bBoxMax.x / 100, bBoxMax.y / 100, bBoxMax.z / 100],
                'CoM':      [x/100, y/100, z/100],
                'Tensor':   t_com.tolist(),
            }

        with open('C:/Users/fish_/Desktop/Igus_{}.yaml'.format(app.activeDocument.name.replace(' ', '_')), 'w') as f:
            f.write(yaml.dump(data))

    except:
        if app.userInterface:
            app.userInterface.messageBox('Failed:\n{}'.format(traceback.format_exc()))
