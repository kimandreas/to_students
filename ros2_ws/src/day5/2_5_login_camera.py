from flask import Flask, render_template, request, redirect, url_for, session, flash, Response
import cv2

app = Flask(__name__)
app.secret_key = 'your_secret_key'  # Needed to secure sessions

# Hardcoded user credentials for demonstration
USERNAME = 'user'
PASSWORD = 'password'

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
    # return render_template('login.html')
    return render_template('login_center.html')

@app.route('/welcome')
def welcome():
    # Ensure user is logged in
    if 'username' not in session:
        flash('Please log in first!', 'warning')
        return redirect(url_for('login'))
    # return render_template('welcome.html', username=session['username'])
    return render_template('welcome_center_cam.html', username=session['username'])

def generate_frames():
    camera = cv2.VideoCapture(0)  # 0 is the default camera

    while True:
        # Read the camera frame
        success, frame = camera.read()
        if not success:
            break
        else:
            # Encode the frame in JPEG format
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # Concatenate frame bytes with multipart data structure
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    # Returns the video stream response
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/logout')
def logout():
    # Clear the session and redirect to login
    session.pop('username', None)
    flash('Logged out successfully!', 'info')
    return redirect(url_for('login'))

if __name__ == "__main__":
    app.run(debug=True)
