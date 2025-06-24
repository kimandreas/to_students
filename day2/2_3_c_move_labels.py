import os
import shutil

def move_corresponding_txt_files(source_dir_with_jpg, source_dir_with_txt, target_dir):
    """
    Moves .txt files corresponding to .jpg files from one directory to another.

    Parameters:
    - source_dir_with_jpg: Directory containing .jpg files.
    - source_dir_with_txt: Directory containing .txt files.
    - target_dir: The directory to move the corresponding .txt files to.
    """
    # Ensure target directory exists
    os.makedirs(target_dir, exist_ok=True)

    # current_directory = os.getcwd()
    # print("current dir=",current_directory)

    # Get list of .jpg filenames in the source directory
    jpg_files = [f for f in os.listdir(source_dir_with_jpg) if f.endswith('.jpg')]

    # Iterate over each .jpg file to find corresponding .txt file
    for jpg_file in jpg_files:
        # Construct .txt filename based on the .jpg filename
        base_filename = os.path.splitext(jpg_file)[0]
        txt_filename = base_filename + '.txt'

        # Construct full path for the .txt file in the source directory
        txt_file_path = os.path.join(source_dir_with_txt, txt_filename)
        # txt_file_path = os.path.join(source_dir_with_txt, 
        #                              txt_filename).replace('\\', '/')
        # print(txt_file_path)


        # Check if the .txt file exists
        if os.path.exists(txt_file_path):
            # Move the .txt file to the target directory
            shutil.copy(txt_file_path, os.path.join(target_dir, txt_filename))
            print(f"Copied: {txt_filename}")
        else:
            print(f"Not found: {txt_filename}")


def main():
    source_dir_with_jpg = './my_data/train/images'
    source_dir_with_txt = './labels'
    target_dir = './my_data/train/labels'

    move_corresponding_txt_files(source_dir_with_jpg, source_dir_with_txt, target_dir)

    source_dir_with_jpg = './my_data/valid/images'
    target_dir = './my_data/valid/labels'

    move_corresponding_txt_files(source_dir_with_jpg, source_dir_with_txt, target_dir)

if __name__ == "__main__":
    main()
