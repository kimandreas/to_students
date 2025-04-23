from flask import Flask, render_template
import sqlite3

app = Flask(__name__)

def create_tables():
    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

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
        
    # Commit the changes and close the connection
    connection.commit()
    connection.close()


def create_detection_entries():

    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

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
        
    # Commit the changes and close the connection
    connection.commit()
    connection.close()


def get_detection_entries():

    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

    # SQL command to select all data from the table
    select_query = "SELECT * FROM detection_table;"

    # Execute the command and fetch all results
    cursor.execute(select_query)
    rows = cursor.fetchall()

    # Print each row
    for row in rows:
        print(row)

    # Commit the changes and close the connection
    connection.commit()
    connection.close()
    return rows
    


# Define route for displaying the detection table data
@app.route('/detection')
def detection_page():
    # Fetch the detection table data
    data = get_detection_entries()
    
    # Render the data in a HTML template
    return render_template('detection.html', data=data)

def main():
    create_tables()
    create_detection_entries()

    
if __name__ == "__main__":


    main()
    app.run(debug=True)



