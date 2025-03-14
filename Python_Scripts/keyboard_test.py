from pynput import keyboard

# Global variables for user input
user_ready = False
within_range = False

def on_key_press(key):
    """
    Handle key presses to set the system state.
    """
    global user_ready
    try:
        if key.char == 's':  # 's' starts balancing only when within range
            user_ready = True
            print("[INFO] User confirmed start of balancing.")
    except AttributeError:
        pass  # Handle special keys like Shift, etc.

def start_keyboard_listener():
    """
    Start the keyboard listener in a separate thread.
    """
    listener = keyboard.Listener(on_press=on_key_press)
    listener.start()

def main():
    start_keyboard_listener()
    while True:
        print(user_ready)
        if user_ready:
            print(user_ready)
            break

if __name__ == "__main__":
    main()