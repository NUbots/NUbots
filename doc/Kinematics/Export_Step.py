# Author-
# Description-
import math
import os
import traceback

import adsk.core
import adsk.fusion
import numpy as np
import yaml


def properties_run(context):
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)

        if not app.activeDocument.name:
            app.userInterface.messageBox("No active Fusion product", "No product")
            return

        # Find all the bodies associated with all of the occurrences in the design
        bodies = []
        for comp in design.allComponents:
            # Get the occurrences that reference this component.
            for occ in design.rootComponent.allOccurrencesByComponent(comp):
                for body in occ.bRepBodies:
                    bodies.append((body, occ.transform))

        data = {}

        # Iterate through all of the bodies in the occurrences.
        for body, transform in bodies:
            physicalProperties = body.physicalProperties
            name = body.parentComponent.name.replace(" ", "_")

            # Get Bounding Box from physical properties
            bBoxMax = body.boundingBox.maxPoint
            bBoxMin = body.boundingBox.minPoint

            # Get the moment of inertia about the world coordinate system.
            (retVal, xx, yy, zz, xy, yz, xz) = physicalProperties.getXYZMomentsOfInertia()

            t_origin = np.asarray([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])

            # Get center of mass from physical properties

            CoM = physicalProperties.centerOfMass
            (x, y, z) = (CoM.x, CoM.y, CoM.z)

            d = np.asarray(
                [
                    [y ** 2 + z ** 2, -x * y, -x * z],
                    [-x * y, x ** 2 + z ** 2, -y * z],
                    [-x * z, -y * z, x ** 2 + y ** 2],
                ]
            )

            t_com = t_origin - physicalProperties.mass * d

            # kg.cm^2 to g.mm^2
            # t_mm = t_com * 1000 * 10 * 10

            # kg.cm^2 to m^2 (unit mass)
            t_com = (t_com / physicalProperties.mass) / 100 / 100

            data[name] = {
                "Area": physicalProperties.area / 100 / 100,
                "Volume": physicalProperties.volume / 100 / 100 / 100,
                "bBoxMin": [bBoxMin.x / 100, bBoxMin.y / 100, bBoxMin.z / 100],
                "bBoxMax": [bBoxMax.x / 100, bBoxMax.y / 100, bBoxMax.z / 100],
                "CoM": [float(x / 100), float(y / 100), float(z / 100)],
                "Tensor": t_com.tolist(),
            }
        with open(
            "C:/Users/fish_/Desktop/Igus_V5/Igus_{}.yaml".format(app.activeDocument.name.replace(" ", "_")), "w"
        ) as f:
            f.write(yaml.dump(data, default_flow_style=None))

    except:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))


def export_step(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)

        # Get the root component of the active design
        rootComp = design.rootComponent

        # Specify the folder to write out the results.
        step_folder = "C:/Users/fish_/Desktop/Igus_V5/step_file/"
        stl_folder = "C:/Users/fish_/Desktop/Igus_V5/stl_file/"

        if not os.path.exists(step_folder):
            os.makedirs(step_folder)

        # Get the root component of the active product.
        rootComp2 = app.activeProduct.rootComponent

        bodies = [body for body in rootComp2.bRepBodies]

        # Iterate through all of the occurrences in the assembly.
        for occ in rootComp2.allOccurrences:
            # Iterate over all of the bodies within the component.
            bodies += [x for x in occ.component.bRepBodies]

        # Iterate through all of the bodies in the occurrences.
        for body in bodies:
            physicalProperties = body.physicalProperties
            name = body.parentComponent.name

            if not os.path.exists((stl_folder + app.activeDocument.name).replace(" ", "_")):
                os.makedirs((stl_folder + app.activeDocument.name).replace(" ", "_"))

            # Construct the output filename.
            filename = (stl_folder + app.activeDocument.name + "/" + name + ".stl").replace(" ", "_")

            # Save the file as STL.
            exportMgr = adsk.fusion.ExportManager.cast(design.exportManager)
            exportOptions = exportMgr.createSTLExportOptions(body)
            exportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            exportOptions.filename = filename
            exportMgr.execute(exportOptions)

        # Construct the output filename.
        filename = (step_folder + app.activeDocument.name + ".step").replace(" ", "_")

        # Save the file as STL.
        exportMgr = adsk.fusion.ExportManager.cast(design.exportManager)
        exportOptions = exportMgr.createSTEPExportOptions(filename)
        exportOptions.filename = filename
        exportMgr.execute(exportOptions)

    except:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)

        properties_run(context)
        # export_step(context)

        ui.messageBox("Finished.")

    except:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
