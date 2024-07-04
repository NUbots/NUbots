#NOTES:

# In the networking md there is a image that it tries to display,
#this is the only working image and you will need to manually move it into the same folder as the .md file.

# In the Nuclear .md file you will need to remove the banner from the code
# as it is corrupt/not displaying as an image.

import glob, os
from pathlib import Path
import shutil, mmap

parent_found = False
folder_num = '01'
results = []
processed = []
#Variables

md_content_first = """---
section: Readmes
chapter: """
md_content_second = """
title: """
md_content_third = """
description: ReadMe files for the """ # description is put into here, just the 5/6th folder name + 4th folder name.
md_content_fourth = """modules in the main NuBots codebase
slug: """
md_content_fifth = """
---

""" # Front matter for MDX files

def check_for_readmes(path):
    global parent_found, results, new_folder_name, path_obj, parent_folder
    for files in glob.glob(path): #Glob (collect) for all files in the directory
        if 'README.md' in files:  #Filter out the files to only include README.md files
            results.append(files)
            if not parent_found: # Checks if a folder on the website already exists for that parent directory and creates it if not
                path_obj = Path(files)
                parent_folder = list(path_obj.parts)[4]
                folder_construct = folder_num + '-' + parent_folder
                new_folder_name = os.path.join('/home/nubots/NUbook/src/book/04-readmes', parent_folder)
                try:
                    os.mkdir(new_folder_name) #Add a new NUbook folder, if one already exists it will pass through
                except FileExistsError:
                    pass
    return
# Checks for any readmes in the specified direcotry, not any sub-directories,
# It also add a master folder to the nu book if there isn't already one for that category.


def find_readmes(parent_path):
    global counter, results, item_path, processed, main_path, parent_found, folder_num
    paths = []
    #results.append(check_for_readmes(parent_path+'*'))
    for item in glob.glob(parent_path+'*/'):
        check_for_readmes(item+'*')
        if item.endswith('/'):
            item_path = list((Path(item)).parts)[-1]
            main_path = list((Path(item)).parts)[-2]
            #print(list((Path(item)).parts))
            try:
                pass
                #os.makedirs('/home/nubots/NUbook/src/book/04-readmes/'+folder_num+'-'+main_path+'/'+item_path+'/')
            except: pass
            for item2 in glob.glob(item+'*/'):
                try:
                    item_path = list((Path(item)).parts)[-1]
                    main_path = list((Path(item)).parts)[-2]
                    #os.makedirs('/home/nubots/NUbook/src/book/04-readmes/'+folder_num+'-'+main_path+'/'+item_path+'/')
                except: pass
                check_for_readmes(item2+'*')
                if item2.endswith('/'): # Goes and checks all files for readme in their name, running i through the check_for_readmes function
                    for item3 in glob.glob(item2+'*/'):
                        check_for_readmes(item3+'*')
                        if item3.endswith('/'):
                            for item4 in glob.glob(item3+'*/'):
                                check_for_readmes(item4+'*')
                                if item4.endswith('/'):
                                    for item5 in glob.glob(item4+'*/'):
                                        check_for_readmes(item5+'*')
                                        if item5.endswith('/'):
                                            for item6 in glob.glob(item5+'*/'):
                                                check_for_readmes(item6+'*')
                                                if item6.endswith('/'):
                                                    for item7 in glob.glob(item6+'*/'):
                                                        check_for_readmes(item7+'*')
                                                        if item7.endswith('/'):
                                                            for item8 in glob.glob(item7+'*/'):
                                                                check_for_readmes(item8+'*')
                                                                if item8.endswith('/'):
                                                                    for item9 in glob.glob(item8+'*/'):
                                                                        check_for_readmes(item9+'*')
                                                                        if item9.endswith('/'):
                                                                            for item10 in glob.glob(item9+'*/'):
                                                                                check_for_readmes(item10+'*')
    parent_folder = ''
    file_maker('', '')
    return

def file_maker(input_f, output_f):
    global results, new_folder_name, path_obj, parent_folder, item_path, main_path
    for file in results:
        if file == 'None' or file in processed:
            pass
        else:
            #print(file)
            processed.append(file)
            with open(results[0], 'r', encoding='utf-8') as f:
                md_content = f.read()

            # Parse front matter and content
            path_obj = Path(results[0])
            slug_obj = Path(file)
            folder4 = list(slug_obj.parts)[4]
            slug = '/readmes/'+folder4+'/'+list(slug_obj.parts)[5] #Gets the slug for the MDX file, based on the file path
            if folder4 == 'module':
                folder4 = ''
            else:
                folder4 = folder4+' '
            markdown_content = md_content_first+parent_folder.title()+md_content_second+list(slug_obj.parts)[-3].title()+md_content_third+list(slug_obj.parts)[-3]+' '+folder4+md_content_fourth+slug+md_content_fifth
            file_path = Path(file)
            item_path2 = list((Path(file)).parts)
            #print(list(file_path.parts))

            name_no_md = list(file_path.parts)[5].removesuffix('.md')
            with open(file,'r') as firstfile, open('/home/nubots/NUbook/src/book/04-readmes/'+list(slug_obj.parts)[4]+'/'+name_no_md+'.mdx','a+') as secondfile:

                try: #To determine if header is already in there, if not add in header
                    file_search = mmap.mmap(secondfile.fileno(), 0, access=mmap.ACCESS_READ)
                    if file_search.find(b'---') != -1:
                        pass
                    else:
                        secondfile.write(markdown_content)
                except ValueError:
                    secondfile.write(markdown_content)

                #Takes out the corrupted banner refrence from the Nuclear readme.
                #If that specific line is found it will be replaced, if not it is written/copied over normally.
                try:
                    first_file_search = mmap.mmap(firstfile.fileno(), 0, access=mmap.ACCESS_READ)
                    filelines = firstfile.readlines()
                    counter = 0
                    for each_line in filelines:
                        counter += 1
                        if each_line.find('your own banner') != -1:
                            print(each_line)
                            secondfile.write('If you do not set your own banner file, !the default banner will be used')
                        else:
                            secondfile.write(each_line)

                except:
                    print('nah')
                '''
                # read content from first file
                for lines in firstfile:
                    # append content to second file
                    secondfile.write(lines)'''
                firstfile.close()
                secondfile.close()
    return

list_of_dirs = ['/home/nubots/NUbots/module', '/home/nubots/NUbots/nuclear', '/home/nubots/NUbots/nusight2', '/home/nubots/NUbots/tools', '/home/nubots/NUbots/roles']
'''results.append('/home/nubots/NUbots/nuclear/README.md')
parent_folder = ''
file_maker('', '')'''
for o in list_of_dirs:
    find_readmes(o) #Runs the find_readmes function on the directories with README files in them
print('YAY')
