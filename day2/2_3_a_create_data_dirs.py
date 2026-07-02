import os
import shutil

def create_directory(directory_name):
    """
    Creates a directory with the given name under the current directory.
    
    Parameters:
    - directory_name: The name of the directory to create.
    
    Returns:
    - None
    """
    # Get the current directory's path
    current_directory = os.getcwd()
    
    # Path for the new directory to be created
    new_directory_path = os.path.join(current_directory, directory_name)
    
    # Create the new directory
    try:
        os.mkdir(new_directory_path)
        print(f"Directory '{directory_name}' created successfully.")
    except FileExistsError:
        print(f"Directory '{directory_name}' already exists.")

# Example usage
create_directory("my_data")
os.chdir('./my_data')

# Delete all files/directories in the my_data directory
for item in os.listdir():
    item_path = os.path.join(os.getcwd(), item)
    
    # Remove directories
    if os.path.isdir(item_path):
        shutil.rmtree(item_path)
        print(f"Deleted directory: {item_path}")
    
    # Remove files
    elif os.path.isfile(item_path):
        os.remove(item_path)
        print(f"Deleted file: {item_path}")


def main():
    create_directory("train")
    create_directory("test")
    create_directory("valid")
    os.chdir('./train')
    create_directory("images")
    create_directory("labels")
    os.chdir('../valid')
    create_directory("images")
    create_directory("labels")
    os.chdir('../test')
    create_directory("images")

if __name__ == "__main__":
    main()



