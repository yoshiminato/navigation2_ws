
import tkinter as tk
from tkinter import simpledialog, filedialog, messagebox
import numpy as np
import os
import argparse

from ament_index_python.packages import get_package_share_directory

world_pkg_share_dir = get_package_share_directory('world_xacro_creator')


class PGMEditor:
    def __init__(self, master, w, h, o, r):
        if not (isinstance(r, (int, float)) and r > 0):
            raise ValueError("resolutionは正の数でなければいけません")
        inv = 1.0 / float(r)
        if abs(inv - round(inv)) > 1e-12:
            raise ValueError(f"resolutionの逆数は整数でなければいけません {inv}")

        self.master = master
        self.w, self.h = w, h
        self.output_path = o
        self.resolution = float(r)
        self.scale = int(1.0 / self.resolution)

        # 固定ウィンドウサイズ
        self.window_size = 600
        self.cell_size = min(self.window_size // max(self.w, self.h), 40)
        self.offset_x = (self.window_size - self.cell_size * self.w) // 2
        self.offset_y = (self.window_size - self.cell_size * self.h) // 2
        self.data = np.zeros((h, w), dtype=np.uint8)
        self.canvas = tk.Canvas(master, width=self.window_size, height=self.window_size, bg='white')
        self.canvas.pack()
        self.rect_start = None
        self.rect_id = None
        self.fill_mode = 1  # 1:黒, 0:白
        self.COEF = 255  # PGMの最大値
        self.canvas.bind("<Button-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.draw()

    def draw(self):
        self.canvas.delete("cell")
        for y in range(self.h):
            for x in range(self.w):
                color = "white" if self.data[y, x] == 0 else "black"
                self.canvas.create_rectangle(
                    self.offset_x + x*self.cell_size, self.offset_y + y*self.cell_size,
                    self.offset_x + (x+1)*self.cell_size, self.offset_y + (y+1)*self.cell_size,
                    fill=color, outline="gray", tags="cell"
                )

    def on_mouse_down(self, event):
        x = (event.x - self.offset_x) // self.cell_size
        y = (event.y - self.offset_y) // self.cell_size
        if not (0 <= x < self.w and 0 <= y < self.h):
            return
        self.rect_start = (x, y)
        self.rect_id = self.canvas.create_rectangle(
            self.offset_x + x*self.cell_size, self.offset_y + y*self.cell_size,
            self.offset_x + x*self.cell_size, self.offset_y + y*self.cell_size,
            outline="red", width=2, tags="rect"
        )

    def on_mouse_drag(self, event):
        if self.rect_id and self.rect_start:
            x0, y0 = self.rect_start
            x1 = (event.x - self.offset_x) // self.cell_size
            y1 = (event.y - self.offset_y) // self.cell_size
            x1 = min(max(x1, 0), self.w)
            y1 = min(max(y1, 0), self.h)
            self.canvas.coords(self.rect_id,
                self.offset_x + x0*self.cell_size, self.offset_y + y0*self.cell_size,
                self.offset_x + x1*self.cell_size, self.offset_y + y1*self.cell_size
            )

    def on_mouse_up(self, event):
        if self.rect_id and self.rect_start:
            x0, y0 = self.rect_start
            x1 = (event.x - self.offset_x) // self.cell_size
            y1 = (event.y - self.offset_y) // self.cell_size
            x1 = min(max(x1, 0), self.w)
            y1 = min(max(y1, 0), self.h)
            x0, x1 = sorted((x0, x1))
            y0, y1 = sorted((y0, y1))
            self.data[y0:y1, x0:x1] = self.fill_mode * self.COEF
            self.canvas.delete(self.rect_id)
            self.rect_id = None
            self.rect_start = None
            self.draw()

    def on_right_click(self, event):
        self.fill_mode ^= 1
        mode = "黒" if self.fill_mode else "白"
        self.master.title(f"PGMエディタ - 塗りつぶしモード: {mode}")

    def save_pgm(self):
        path = os.path.join(world_pkg_share_dir, 'maps', f'{self.output_path}.pgm')
        scale = self.scale
        expanded_arr = np.repeat(np.repeat(self.data, repeats=scale, axis=0), repeats=scale, axis=1)
       
        try:
            with open(path, "w") as f:
                f.write(f"P2\n{self.w*scale} {self.h*scale}\n255\n")
                for row in expanded_arr:
                    # 255-値で反転して保存（0=黒=壁, 255=白=空間）
                    f.write(" ".join(str(255 - int(v)) for v in row) + "\n")
            messagebox.showinfo("保存完了", f"保存しました: {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("保存エラー", str(e))

        os._exit(0)


if __name__ == "__main__":
    root = tk.Tk()
    root.title("PGMエディタ - 塗りつぶしモード: 黒")

    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, default=20, help='PGMの幅（ピクセル数）')
    parser.add_argument('--height', type=int, default=20, help='PGMの高さ（ピクセル数）')
    parser.add_argument('--world_name', type=str, default='map', help='出力PGMファイル名')
    parser.add_argument('--resolution', type=float, default=0.05, help='マップの解像度（メートル/ピクセル）')
    args = parser.parse_args()

    w = args.width
    h = args.height
    o = args.world_name
    r = args.resolution

    # サイズ入力ダイアログ
    # w = simpledialog.askinteger("幅", "幅（ピクセル数）", minvalue=1, maxvalue=200)
    # h = simpledialog.askinteger("高さ", "高さ（ピクセル数）", minvalue=1, maxvalue=200)
    if w and h:
        try:
            app = PGMEditor(root, w, h, o, r)
        except ValueError as e:
            messagebox.showerror("入力エラー", str(e))
            root.destroy()
        else:
            # メニューバー（app を作成してから登録）
            menubar = tk.Menu(root)
            filemenu = tk.Menu(menubar, tearoff=0)
            filemenu.add_command(label="保存", command=app.save_pgm)
            filemenu.add_separator()
            filemenu.add_command(label="終了", command=root.quit)
            menubar.add_cascade(label="ファイル", menu=filemenu)
            root.config(menu=menubar)

            root.mainloop()