import tkinter as tk
from tkinter import Label, Scrollbar, Canvas, Frame
from PIL import Image, ImageTk
import os
import subprocess


def on_image_click(image_name):
    """
    Handler for when an image is clicked. Executes a separate Python script,
    passing the image name as an argument.

    Parameters:
    - image_name (str): The name of the clicked image.
    """
    # Build the command to execute the script with the image name as an argument
    print("investigating:", image_name)
    command = f'python3 visualize_tags.py "{image_name}"'
    subprocess.run(command, shell=True)


def display_images(image_paths, query):
    """
    Displays images in a Tkinter window with scroll functionality.

    Parameters:
    - image_paths (list): A list of paths to the images to be displayed.
    - query (str): The search query to display as the window title.
    """
    root = tk.Tk()
    root.title("Image Results for query: {}".format(query))

    # Set window size
    window_width = 1280
    window_height = 720
    root.geometry(f"{window_width}x{window_height}")

    # Header label for the query
    header_label = Label(
        root, text="Image Results for query: " + query.upper(), font=('Arial', 20))
    header_label.pack(fill=tk.X)

    # Create a canvas and a scrollbar
    canvas = Canvas(root, width=window_width, height=window_height)
    scrollbar = Scrollbar(root, command=canvas.yview)
    canvas.configure(yscrollcommand=scrollbar.set)

    # Pack the scrollbar and canvas
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # Frame inside canvas
    frame = Frame(canvas)
    canvas.create_window((0, 0), window=frame, anchor='nw')

    # Image and label layout parameters
    img_width, img_height = 250, 250  # Desired image display size
    padding = 10  # Padding between images
    images_per_row = window_width // (img_width + padding)

    # Adjust row calculation to account for labels
    def calculate_position(index):
        """
        Calculates the grid position for an image based on its index.

        Parameters:
        - index (int): The index of the image in the list.

        Returns:
        - Tuple (row, col): The row and column in the grid where the image should be placed.
        """
        return (index // images_per_row) * 2, index % images_per_row

    def on_enter(event):
        """Change cursor to hand on hover over image."""
        root.config(cursor="hand2")

    def on_leave(event):
        """Revert cursor to default when not hovering over image."""
        root.config(cursor="")

    # Display images in the frame
    for idx, image_path in enumerate(image_paths):
        row, col = calculate_position(idx)
        img = Image.open(image_path)
        img = img.resize((img_width, img_height), Image.ANTIALIAS)
        photo = ImageTk.PhotoImage(img)
        label_image = Label(frame, image=photo)
        label_image.image = photo  # Keep a reference!
        label_image.grid(row=row, column=col, padx=padding, pady=padding)

        # Bind click event to the image
        image_name = os.path.basename(image_path)
        label_image.bind("<Button-1>", lambda event,
                         name=image_name: on_image_click(name))

        # Bind enter and leave events for hover effect
        label_image.bind("<Enter>", on_enter)
        label_image.bind("<Leave>", on_leave)

        # Display image name below the image
        label_name = Label(frame, text=image_name)
        label_name.grid(row=row+1, column=col, padx=padding, pady=0)

    frame.update_idletasks()
    canvas.config(scrollregion=canvas.bbox("all"))

    root.mainloop()


def parse_log_file(log_file_path):
    """
    Parses a log file to extract image paths and display them.

    Parameters:
    - log_file_path (str): Path to the log file.
    """
    image_paths = []
    try:
        with open(log_file_path, 'r') as file:
            lines = file.readlines()
        # Reverse the file content to find the most recent entry first
        print("File contains {} lines".format(len(lines)))

        # to get the most recent entry, the second time > is the starting character, we stop
        chevron_count = 0
        query = None

        for line in reversed(lines):
            if line.strip().startswith('* index='):
                # Extract the image path from the line
                print(line)
                image_path = line.split(' ')[6].strip().split('/')[2]
                # Add the full path to the list, adjust the base path as needed
                image_paths.append(os.path.join(
                    '/data/datasets/image_collector/train', image_path))
                print("Added image path: {}".format(image_path))
            # Stop at the first query marker
            if line.strip().startswith('>'):
                chevron_count += 1
                if chevron_count == 2:
                    query = line[2:]
                    print(query)
                    break
        # Display the images if any paths were found
        if image_paths:
            # Reverse back to original order for display
            display_images(image_paths[::-1], query)
    except FileNotFoundError as e:
        print(e)


log_file_path = 'out.txt'
parse_log_file(log_file_path)
