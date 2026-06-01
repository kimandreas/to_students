import os
import shutil
import time
import random

def split_files(source_directory, train_dir="./my_data/train/images", 
                validation_dir="./my_data/valid/images", 
                test_dir="./my_data/test/images", 
                train_ratio=0.7, validation_ratio=0.2):
    """
    Splits files in the source directory into train, validation, and test directories.

    Parameters:
    - source_directory: Directory containing files to split.
    - train_dir: Name of the directory for training files.
    - valid_dir: Name of the directory for validation files.
    - test_dir: Name of the directory for test files.
    - train_ratio: Proportion of files to be used for training.
    - validation_ratio: Proportion of files to be used for validation.
    """
    
    # Ensure ratios sum to 1
    if (train_ratio + validation_ratio) > 1:
        print("Error: Ratios sum to more than 1.")
        return

    # Create destination directories if they do not exist
    for dir_name in [train_dir, validation_dir, test_dir]:
        # os.makedirs(os.path.join(source_directory, dir_name), exist_ok=True)
        os.makedirs(os.path.join(dir_name), exist_ok=True)

    # Get all files in the source directory
    all_files = [f for f in os.listdir(source_directory) if os.path.isfile(os.path.join(source_directory, f))]
    
    random.seed(time.time())
    random.shuffle(all_files)  # Shuffle to randomize file selection

    # Calculate split indices
    total_files = len(all_files)
    train_end = int(total_files * train_ratio)
    validation_end = train_end + int(total_files * validation_ratio)

    # Split files
    train_files = all_files[:train_end]
    validation_files = all_files[train_end:validation_end]
    test_files = all_files[validation_end:]

    # Function to copy files to destination directories
    def copy_files(files, destination):
        for file in files:
            shutil.copy(os.path.join(source_directory, file), 
                        os.path.join(destination, file))

    # Copy files to their respective directories
    copy_files(train_files, train_dir)
    copy_files(validation_files, validation_dir)
    copy_files(test_files, test_dir)

    print(f"Files split into {train_dir}, {validation_dir}, and {test_dir} directories.")

def main():
    source_directory = input("Enter directory path to saved images: ")
    split_files(source_directory)

if __name__ == "__main__":
    main()

