import subprocess
import time
import socket
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart


def check_internet_connection():
    try:
        # Attempt to resolve the Google DNS server
        socket.gethostbyname('dns.google')
        return True
    except socket.error:
        return False


def get_local_ip_address():
    command = "ip a"
    result = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = result.communicate()[0].decode()

    # Find the line containing the IP address information
    for line in output.splitlines():
        if 'inet' in line and 'wlan0' in line:
            # Extract the IP address from the line
            ip_address = line.strip().split()[1].split('/')[0]
            return ip_address


def wait_for_internet_connection():
    print("Waiting for internet connection...")

    while not check_internet_connection():
        time.sleep(1)

    print("Internet connection established!")


def send_email(sender_email, sender_password, receiver_emails, subject, body, important=False):
    # Create a multipart message
    message = MIMEMultipart()
    message["From"] = sender_email
    message["To"] = ", ".join(receiver_emails)
    message["Subject"] = subject

    # Add body to the email
    message.attach(MIMEText(body, "plain"))

    if important:
        # Set the importance header
        message["Importance"] = "High"

    try:
        # Set up the SMTP server
        smtp_server = smtplib.SMTP("smtp.gmail.com", 587)
        smtp_server.starttls()
        smtp_server.login(sender_email, sender_password)

        # Send the email
        smtp_server.sendmail(sender_email, receiver_emails, message.as_string())
        print("Email sent successfully!")

    except smtplib.SMTPException as e:
        print("Error occurred while sending email:", str(e))

    finally:
        smtp_server.quit()


wait_for_internet_connection()
local_ip = None
local_ip = get_local_ip_address()

print("Local IP address:", local_ip)

# Email configuration
sender_email = "travisnocat@gmail.com"
sender_password = "dbwrtoevucdxbdys"
receiver_emails = ["carpit680@gmail.com", "hunar.islam01@gmail.com"]  # Add your desired receiver email addresses here
subject = "Local IP Address"
body = f"The local IP address on the WiFi network is: {local_ip} \n Use the following to connect to ssh remotely then vnc to localhost:5900 \n $ ssh -L 5900:localhost:5900 hunar@7.tcp.ngrok.io -p 20432"
important = True  # Set to True if you want to mark the email as important

send_email(sender_email, sender_password, receiver_emails, subject, body, important)
