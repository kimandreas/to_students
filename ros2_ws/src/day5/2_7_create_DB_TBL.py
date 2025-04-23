import sqlite3

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

# Commit the changes and close the connection
connection.commit()
connection.close()

print("Database and table created successfully.")
