"""

    groupcollapsible {
        name        "auto_folder"
        label       "Auto Fill"
        grouptag    { "group_type" "simple" }
        parmtag     { "group_default" "1" }

        parm {
            name    "folder"
            label   "Folder"
            type    directory
            default { "" }
        }
        parm {
            name    "substeps"
            label   "Substeps"
            type    int
            default { "1" }
        }
        parm {
            name    "autoFillBtn"
            label   "Auto Fill From Folder"
            type    button
            parmtag { "script_callback" "import os; node = kwargs['node']; foldername = node.parm('folder').eval(); substeps = node.parm('substeps').eval(); file_multiparm = node.parm('numfiles'); file_childparmname = 'filename#'; files = []; exec('for f in os.listdir(foldername):\n\tname, ext = os.path.splitext(f)\nif os.path.isfile(f\'{foldername}/{f}\') and ext == \'.txt\':\n\tfiles.append(f)'); sorted_files = sorted(files); substep_files = sorted_files[::substeps]; data = [{file_childparmname: f} for f in substep_files]; file_multiparm.setMultiParmInstancesFromData(data)" }
            parmtag { "script_callback_language" "python" }
        }
    }
"""

import os



def import_files(node, foldername, substeps):
    file_multiparm = node.parm("numfiles")
    file_childparmname = "filename#"

    files = []
    for f in os.listdir(foldername):
        name, ext = os.path.splitext(f)
        path = f"{foldername}/{f}"
        if os.path.isfile(path) and ext == '.txt':
            files.append(path)

    sorted_files = sorted(files)
    substep_files = sorted_files[::substeps]

    data = [{file_childparmname: f} for f in substep_files]
    print(f"Setting f{file_multiparm} to f{data}")
    file_multiparm.setMultiParmInstancesFromData(data)

node = kwargs['node']
foldername = node.parm("folder").eval()
substeps = node.parm("substeps").eval()

# import os; node = kwargs['node']; foldername = node.parm('folder').eval(); substeps = node.parm('substeps').eval(); file_multiparm = node.parm('numfiles'); file_childparmname = 'filename#'; files = []; exec('for f in os.listdir(foldername):\n\tname, ext = os.path.splitext(f)\nif os.path.isfile(f\'{foldername}/{f}\') and ext == \'.txt\':\n\tfiles.append(f)'); sorted_files = sorted(files); substep_files = sorted_files[::substeps]; data = [{file_childparmname: f} for f in substep_files]; file_multiparm.setMultiParmInstancesFromData(data)

