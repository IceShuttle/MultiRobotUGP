import tkinter as tk
from tkinter import messagebox
import csv
import os

class BinaryCSVPainter:
    def __init__(self, rows=16, cols=24, cell_size=25):
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.grid = [[0] * cols for _ in range(rows)]

        self.root = tk.Tk()
        self.root.title("CSV Binary Painter (Drag to Draw)")
        self.root.resizable(False, False)

        # Drawing canvas
        self.canvas = tk.Canvas(
            self.root, 
            width=cols * cell_size, 
            height=rows * cell_size,
            bg="white", 
            highlightthickness=1, 
            highlightbackground="#aaa"
        )
        self.canvas.pack(padx=10, pady=5)

        self.draw_grid()

        # Bind drawing/erasing
        self.canvas.bind("<Button-1>", self.paint)
        self.canvas.bind("<B1-Motion>", self.paint)
        self.canvas.bind("<Button-3>", self.erase)
        self.canvas.bind("<B3-Motion>", self.erase)

        # Suppress default OS right-click menu on canvas
        self.canvas.bind("<Button-3>", lambda e: "break")

        # Controls
        ctrl_frame = tk.Frame(self.root)
        ctrl_frame.pack(pady=5)
        tk.Button(ctrl_frame, text="💾 Save CSV", command=self.save_csv, font=("Arial", 10, "bold")).pack(side="left", padx=5)
        tk.Button(ctrl_frame, text="🗑 Clear", command=self.clear_grid, font=("Arial", 10)).pack(side="left", padx=5)

        tk.Label(self.root, text="🖱 Left-click/drag: Draw (1)  |  Right-click/drag: Erase (0)").pack(pady=5)

    def draw_grid(self):
        self.rects = []
        for r in range(self.rows):
            row = []
            for c in range(self.cols):
                x1, y1 = c * self.cell_size, r * self.cell_size
                x2, y2 = x1 + self.cell_size, y1 + self.cell_size
                rect = self.canvas.create_rectangle(x1, y1, x2, y2, fill="white", outline="#ddd")
                row.append(rect)
            self.rects.append(row)

    def get_cell(self, x, y):
        c, r = int(x // self.cell_size), int(y // self.cell_size)
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return r, c
        return None, None

    def paint(self, event):
        r, c = self.get_cell(event.x, event.y)
        if r is not None and self.grid[r][c] == 0:
            self.grid[r][c] = 1
            self.canvas.itemconfig(self.rects[r][c], fill="black")

    def erase(self, event):
        r, c = self.get_cell(event.x, event.y)
        if r is not None and self.grid[r][c] == 1:
            self.grid[r][c] = 0
            self.canvas.itemconfig(self.rects[r][c], fill="white")

    def clear_grid(self):
        for r in range(self.rows):
            for c in range(self.cols):
                self.grid[r][c] = 0
                self.canvas.itemconfig(self.rects[r][c], fill="white")

    def save_csv(self):
        filename = "binary_drawing.csv"
        with open(filename, "w", newline="") as f:
            csv.writer(f).writerows(self.grid)
        messagebox.showinfo("Saved!", f"Grid exported to:\n{os.path.abspath(filename)}")

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    # Adjust rows, cols, and cell_size as needed
    BinaryCSVPainter(rows=30, cols=50, cell_size=30).run()
