import sqlite3

# Connect to SQLite database (or create it if it doesn't exist)
connection = sqlite3.connect('mydatabase.db')

# Create a cursor object to interact with the database
cursor = connection.cursor()

def create_tables():
    # SQL command to create the table
    create_detection_table = """
    CREATE TABLE IF NOT EXISTS detection_table (
        id INTEGER PRIMARY KEY,
        name TEXT NOT NULL
    );
    """

    # SQL command to create the table
    create_violation_table = """
    CREATE TABLE IF NOT EXISTS violation_detected (
        id INTEGER PRIMARY KEY,
        name TEXT NOT NULL,
        time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

    # Execute the command
    cursor.execute(create_detection_table)
    cursor.execute(create_violation_table)

    # SQL command to delete all entries in the detection_table
    delete_all_entries_query = "DELETE FROM detection_table;"

    # Execute the command
    cursor.execute(delete_all_entries_query)

        # SQL command to delete all entries in the detection_table
    delete_all_entries_query = "DELETE FROM violation_detected;"

    # Execute the command
    cursor.execute(delete_all_entries_query)

    print("Tables created and emptied successfully.")

def create_Detection_entries():
    # Data to insert (a list of tuples, where each tuple represents a row)
    detection_entries = [
        (0,'Truck'),
        (1,'Dummy'),
    ]

    # SQL command to insert data
    insert_query = """
    INSERT INTO detection_table (id, name) VALUES (?, ?);
    """
    cursor.executemany(insert_query,detection_entries)

    # SQL command to select all data from the table
    select_query = "SELECT * FROM detection_table;"

    # Execute the command and fetch all results
    cursor.execute(select_query)
    rows = cursor.fetchall()

    # Print each row
    for row in rows:
        print(row)

def main():
    create_tables()
    create_Detection_entries()
    
if __name__ == "__main__":

    main()

    # Commit the changes and close the connection
    connection.commit()
    connection.close()


