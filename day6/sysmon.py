from flask import Flask, render_template, request, redirect, url_for, session, flash, Response
import cv2
import numpy as np
import sqlite3
from datetime import datetime

app = Flask(__name__)
app.secret_key = 'your_secret_key'  # Needed to secure sessions

# Hardcoded user credentials for demonstration
USERNAME = 'user'
PASSWORD = 'password'

def create_DB_tables():
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
    CREATE TABLE IF NOT EXISTS violation_table (
        id INTEGER PRIMARY KEY,
        name TEXT NOT NULL,
        date_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

    # SQL command to create the table
    create_tracking_table = """
    CREATE TABLE IF NOT EXISTS tracking_table (
        id INTEGER PRIMARY KEY,
        name TEXT NOT NULL,
        date_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """

    # Execute the command
    cursor.execute(create_detection_table)
    cursor.execute(create_violation_table)
    cursor.execute(create_tracking_table)

    # SQL command to delete all entries in the detection_table
    delete_all_entries_query = "DELETE FROM detection_table;"
    cursor.execute(delete_all_entries_query)

    # SQL command to delete all entries in the violation_table
    delete_all_entries_query = "DELETE FROM violation_table;"
    cursor.execute(delete_all_entries_query)

    # SQL command to delete all entries in the tracking_table
    delete_all_entries_query = "DELETE FROM tracking_table;"
    cursor.execute(delete_all_entries_query)

    print("DB and Tables created and emptied successfully.")
        
    # Commit the changes and close the connection
    connection.commit()
    connection.close()


def create_detection_entries():

    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

    # insert global object to detect (a list of tuples, where each tuple represents a row)
    detection_entries = [
        (0,'Truck'),
        (1,'Dummy')
    ]

    insert_query = """
    INSERT INTO detection_table (id, name) VALUES (?, ?);
    """
    cursor.executemany(insert_query,detection_entries)
        
    # Commit the changes and close the connection
    connection.commit()
    connection.close()

def create_violation_entries():

    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

    # Data to insert (a list of tuples, where each tuple represents a row)
    date_time_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S") 

    violation_entries = [
        (0,'Truck', date_time_now),
        # (0,'Truck',)
    ]

    # SQL command to insert data
    insert_query = """
    INSERT INTO violation_table (id, name, date_time) VALUES (?, ?, ?);
    """
    cursor.executemany(insert_query,violation_entries)
        
    # Commit the changes and close the connection
    connection.commit()
    connection.close()

def create_tracking_entries():

    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

    # Data to insert (a list of tuples, where each tuple represents a row)
    date_time_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S") 

    tracking_entries = [
        (1,'Dummy', date_time_now),
        # (1,'Dummy',)
    ]

    # SQL command to insert data
    insert_query = """
    INSERT INTO tracking_table (id, name, date_time) VALUES (?, ?, ?);
    """
    cursor.executemany(insert_query,tracking_entries)
        
    # Commit the changes and close the connection
    connection.commit()
    connection.close()

def get_violation_entries():

    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

    # SQL command to select all data from the table
    select_query = "SELECT * FROM violation_table;"

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

def get_tracking_entries():

    # Connect to SQLite database (or create it if it doesn't exist)
    connection = sqlite3.connect('mydatabase.db')

    # Create a cursor object to interact with the database
    cursor = connection.cursor()

    # SQL command to select all data from the table
    select_query = "SELECT * FROM tracking_table;"

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

@app.route('/')
def home():
    # If user is logged in, redirect to welcome page
    if 'username' in session:
        return redirect(url_for('welcome'))
    # Otherwise, show the login page
    return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        # Retrieve form data
        username = request.form['username']
        password = request.form['password']
        
        # Check credentials
        if username == USERNAME and password == PASSWORD:
            # Store username in session and redirect to welcome
            session['username'] = username
            flash('Login successful!', 'success')
            return redirect(url_for('welcome'))
        else:
            flash('Invalid username or password!', 'danger')
            return redirect(url_for('login'))
    
    # Display login page for GET requests
    return render_template('login_center.html')

@app.route('/welcome')
def welcome():

    # Ensure user is logged in
    if 'username' not in session:
        flash('Please log in first!', 'warning')
        return redirect(url_for('login'))
    
    # Fetch the detection table data
    data1 = get_violation_entries()
    data2 = get_tracking_entries()

    return render_template('sysmon.html', username=session['username'], data1=data1, data2=data2)

def generate_frames(camera_id):
    camera = cv2.VideoCapture(camera_id)  # 0 is the default camera

    while True:
        # Read the camera frame
        success, frame = camera.read()
        if not success:
            break
        else:
            # if 
            #     img=cv2.bitwise_not(img)

            # Encode the frame in JPEG format
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # Concatenate frame bytes with multipart data structure

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# List of store coordinates
pt_1 = (460, 0)
pt_2 = (640, 0)
pt_3 = (640, 120)
pt_4 = (460, 120)
coordinates = [pt_1, pt_2, pt_3, pt_4]

def generate_frames_box(camera_id):
    camera = cv2.VideoCapture(camera_id)

    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Draw each collected coordinate on the frame
            for (x, y) in coordinates:
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Red circle with radius 5
            
            # Convert coordinates to the format required by cv2.polylines
            pts = np.array(coordinates, np.int32)
            pts = pts.reshape((-1, 1, 2))

            # Draw the quadrilateral
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed1')
def video_feed1():
    return Response(generate_frames_box(0), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    return Response(generate_frames(1), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/logout')
def logout():
    # Clear the session and redirect to login
    session.pop('username', None)
    flash('Logged out successfully!', 'info')
    return redirect(url_for('login'))

if __name__ == "__main__":
    create_DB_tables()
    create_detection_entries()
    create_violation_entries()
    create_tracking_entries()
    app.run(debug=True)
