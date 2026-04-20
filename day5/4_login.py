from flask import Flask, render_template, request, redirect, url_for, session, flash

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
    return render_template('welcome_center.html', username=session['username'])

@app.route('/logout')
def logout():
    # Clear the session and redirect to login
    session.pop('username', None)
    flash('Logged out successfully!', 'info')
    return redirect(url_for('login'))

if __name__ == "__main__":
    app.run(debug=True)
