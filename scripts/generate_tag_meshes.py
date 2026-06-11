#!/usr/bin/env python3

import argparse
import os
import shutil
import subprocess
import tempfile

import yaml

try:
    import tkinter as tk
    from tkinter import messagebox
    from PIL import Image
except ImportError as error:
    raise SystemExit(
        "generate_tag_meshes.py requires tkinter and Pillow. "
        "On Ubuntu/ROS Noetic, install python3-tk and python3-pil. "
        "Import error: %s" % error
    )


DAE_TEMPLATE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_images>
    <image id="tag_image" name="tag_image">
      <init_from>{texture_path}</init_from>
    </image>
  </library_images>
  <library_effects>
    <effect id="tag_effect">
      <profile_COMMON>
        <newparam sid="tag_surface">
          <surface type="2D">
            <init_from>tag_image</init_from>
          </surface>
        </newparam>
        <newparam sid="tag_sampler">
          <sampler2D>
            <source>tag_surface</source>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <lambert>
            <diffuse>
              <texture texture="tag_sampler" texcoord="UVSET0"/>
            </diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="tag_material" name="tag_material">
      <instance_effect url="#tag_effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="tag_plane" name="tag_plane">
      <mesh>
        <source id="tag_plane-positions">
          <float_array id="tag_plane-positions-array" count="12">-0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5 0</float_array>
          <technique_common>
            <accessor source="#tag_plane-positions-array" count="4" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="tag_plane-uv">
          <float_array id="tag_plane-uv-array" count="8">0 0 1 0 1 1 0 1</float_array>
          <technique_common>
            <accessor source="#tag_plane-uv-array" count="4" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="tag_plane-vertices">
          <input semantic="POSITION" source="#tag_plane-positions"/>
        </vertices>
        <triangles material="tag_material" count="2">
          <input semantic="VERTEX" source="#tag_plane-vertices" offset="0"/>
          <input semantic="TEXCOORD" source="#tag_plane-uv" offset="1" set="0"/>
          <p>0 0 1 1 2 2 0 0 2 2 3 3</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="tag_node" name="tag_node">
        <instance_geometry url="#tag_plane">
          <bind_material>
            <technique_common>
              <instance_material symbol="tag_material" target="#tag_material">
                <bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""


def package_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def default_tag_dir():
    return os.path.join(package_root(), "assets", "tags")


def default_mesh_dir():
    return os.path.join(package_root(), "assets", "meshes")


def crop_yaml_path(source_path, page=None):
    base, _ = os.path.splitext(source_path)
    if page is not None:
        return base + ".page_%d.crops.yaml" % page
    return base + ".crops.yaml"


def normalize_rect(x0, y0, x1, y1):
    left = min(x0, x1)
    top = min(y0, y1)
    right = max(x0, x1)
    bottom = max(y0, y1)
    return int(left), int(top), int(right), int(bottom)


def write_dae(mesh_path, texture_path):
    rel_texture = os.path.relpath(texture_path, os.path.dirname(mesh_path))
    rel_texture = rel_texture.replace(os.sep, "/")
    with open(mesh_path, "w") as dae_file:
        dae_file.write(DAE_TEMPLATE.format(texture_path=rel_texture))


class TagCropApp:
    def __init__(self, root, source_path, image_path, tag_dir, mesh_dir, page=None, temp_paths=None):
        self.root = root
        self.source_path = os.path.abspath(source_path)
        self.image_path = os.path.abspath(image_path)
        self.tag_dir = os.path.abspath(tag_dir)
        self.mesh_dir = os.path.abspath(mesh_dir)
        self.page = page
        self.temp_paths = temp_paths or []
        self.crops_path = crop_yaml_path(self.source_path, page)
        self.crops = []
        self.current_rect_id = None
        self.drag_start = None
        self.pending_rect = None
        self.image_item = None

        self.source_image = Image.open(self.image_path)
        self.display_scale = 1.0
        self.display_width = self.source_image.width
        self.display_height = self.source_image.height
        self.display_temp = tempfile.NamedTemporaryFile(prefix="apriltag_mesh_source_", suffix=".png", delete=False)
        self.display_temp.close()
        self.photo = None

        self.root.title("AprilTag mesh generator")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.build_ui()
        self.update_display_image(1000, 720)
        self.load_crops()
        self.redraw_rectangles()
        self.refresh_list()

    def build_ui(self):
        main = tk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main, width=1000, height=720, bg="gray20")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.image_item = self.canvas.create_image(0, 0, anchor=tk.NW)
        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Configure>", self.on_canvas_configure)

        side = tk.Frame(main, padx=8, pady=8)
        side.grid(row=0, column=1, sticky="ns")

        tk.Label(side, text="Tag ID").pack(anchor=tk.W)
        self.tag_id_entry = tk.Entry(side, width=12)
        self.tag_id_entry.pack(anchor=tk.W, fill=tk.X)
        self.tag_id_entry.insert(0, "0")

        tk.Button(side, text="Add / Update", command=self.add_or_update_crop).pack(fill=tk.X, pady=(8, 0))
        tk.Button(side, text="Delete Selected", command=self.delete_selected).pack(fill=tk.X, pady=(4, 0))
        tk.Button(side, text="Generate", command=self.generate).pack(fill=tk.X, pady=(16, 0))

        self.listbox = tk.Listbox(side, width=36, height=20)
        self.listbox.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.listbox.bind("<<ListboxSelect>>", self.on_select)

        main.columnconfigure(0, weight=1)
        main.rowconfigure(0, weight=1)

    def update_display_image(self, available_width, available_height):
        available_width = max(1, available_width)
        available_height = max(1, available_height)
        self.display_scale = min(
            float(available_width) / float(self.source_image.width),
            float(available_height) / float(self.source_image.height),
        )
        self.display_width = max(1, int(self.source_image.width * self.display_scale))
        self.display_height = max(1, int(self.source_image.height * self.display_scale))

        display_image = self.source_image.resize((self.display_width, self.display_height), Image.NEAREST)
        display_image.save(self.display_temp.name)
        self.photo = tk.PhotoImage(file=self.display_temp.name)
        self.canvas.itemconfigure(self.image_item, image=self.photo)
        self.canvas.configure(scrollregion=(0, 0, self.display_width, self.display_height))

    def on_canvas_configure(self, event):
        if event.width <= 1 or event.height <= 1:
            return
        previous_width = getattr(self, "display_width", None)
        previous_height = getattr(self, "display_height", None)
        self.update_display_image(event.width, event.height)
        if self.display_width != previous_width or self.display_height != previous_height:
            self.redraw_rectangles()
            self.redraw_pending_rectangle()

    def on_close(self):
        if hasattr(self, "display_temp") and os.path.exists(self.display_temp.name):
            os.unlink(self.display_temp.name)
        for path in self.temp_paths:
            if os.path.exists(path):
                if os.path.isdir(path):
                    shutil.rmtree(path)
                else:
                    os.unlink(path)
        self.root.destroy()

    def image_coords(self, event):
        x = max(0, min(self.source_image.width, event.x / self.display_scale))
        y = max(0, min(self.source_image.height, event.y / self.display_scale))
        return x, y

    def canvas_coords(self, rect):
        x = rect["x"] * self.display_scale
        y = rect["y"] * self.display_scale
        return x, y, (rect["x"] + rect["width"]) * self.display_scale, (rect["y"] + rect["height"]) * self.display_scale

    def redraw_pending_rectangle(self):
        if not self.pending_rect:
            return
        x0, y0, x1, y1 = self.pending_rect
        coords = (
            x0 * self.display_scale,
            y0 * self.display_scale,
            x1 * self.display_scale,
            y1 * self.display_scale,
        )
        if self.current_rect_id:
            self.canvas.coords(self.current_rect_id, *coords)
        else:
            self.current_rect_id = self.canvas.create_rectangle(
                *coords,
                outline="cyan",
                width=2,
            )

    def on_press(self, event):
        self.drag_start = self.image_coords(event)
        if self.current_rect_id:
            self.canvas.delete(self.current_rect_id)
            self.current_rect_id = None

    def on_drag(self, event):
        if not self.drag_start:
            return
        start_x, start_y = self.drag_start
        end_x, end_y = self.image_coords(event)
        x0, y0, x1, y1 = normalize_rect(start_x, start_y, end_x, end_y)
        if self.current_rect_id:
            self.pending_rect = (x0, y0, x1, y1)
            self.redraw_pending_rectangle()
        else:
            self.pending_rect = (x0, y0, x1, y1)
            self.current_rect_id = self.canvas.create_rectangle(
                x0 * self.display_scale,
                y0 * self.display_scale,
                x1 * self.display_scale,
                y1 * self.display_scale,
                outline="cyan",
                width=2,
            )

    def on_release(self, event):
        if not self.drag_start:
            return
        start_x, start_y = self.drag_start
        end_x, end_y = self.image_coords(event)
        self.pending_rect = normalize_rect(start_x, start_y, end_x, end_y)
        self.drag_start = None

    def add_or_update_crop(self):
        tag_id_text = self.tag_id_entry.get().strip()
        if not tag_id_text.isdigit():
            messagebox.showerror("Invalid tag id", "Tag ID must be a non-negative integer.")
            return
        if not self.pending_rect:
            messagebox.showerror("No rectangle", "Drag a rectangle on the source image first.")
            return

        x0, y0, x1, y1 = self.pending_rect
        if x1 - x0 < 2 or y1 - y0 < 2:
            messagebox.showerror("Invalid rectangle", "Selected rectangle is too small.")
            return

        crop = {
            "id": int(tag_id_text),
            "rect": {
                "x": x0,
                "y": y0,
                "width": x1 - x0,
                "height": y1 - y0,
            },
        }

        for index, existing in enumerate(self.crops):
            if existing["id"] == crop["id"]:
                self.crops[index] = crop
                break
        else:
            self.crops.append(crop)

        self.crops.sort(key=lambda item: item["id"])
        self.save_crops()
        self.redraw_rectangles()
        self.refresh_list()

    def delete_selected(self):
        selection = self.listbox.curselection()
        if not selection:
            return
        index = selection[0]
        del self.crops[index]
        self.save_crops()
        self.redraw_rectangles()
        self.refresh_list()

    def on_select(self, _event):
        selection = self.listbox.curselection()
        if not selection:
            return
        crop = self.crops[selection[0]]
        self.tag_id_entry.delete(0, tk.END)
        self.tag_id_entry.insert(0, str(crop["id"]))
        rect = crop["rect"]
        self.pending_rect = normalize_rect(rect["x"], rect["y"], rect["x"] + rect["width"], rect["y"] + rect["height"])

    def refresh_list(self):
        self.listbox.delete(0, tk.END)
        for crop in self.crops:
            rect = crop["rect"]
            self.listbox.insert(
                tk.END,
                "tag_%s  x=%s y=%s w=%s h=%s" % (
                    crop["id"],
                    rect["x"],
                    rect["y"],
                    rect["width"],
                    rect["height"],
                ),
            )

    def redraw_rectangles(self):
        self.canvas.delete("crop_rect")
        self.canvas.delete("crop_label")
        for crop in self.crops:
            rect = crop["rect"]
            x0, y0, x1, y1 = self.canvas_coords(rect)
            self.canvas.create_rectangle(x0, y0, x1, y1, outline="yellow", width=2, tags="crop_rect")
            self.canvas.create_text(
                x0 + 4,
                y0 + 4,
                text="tag_%s" % crop["id"],
                anchor=tk.NW,
                fill="yellow",
                tags="crop_label",
            )

    def load_crops(self):
        if not os.path.exists(self.crops_path):
            return
        with open(self.crops_path, "r") as crops_file:
            data = yaml.safe_load(crops_file) or {}
        self.crops = data.get("tags", [])

    def save_crops(self):
        data = {
            "source": os.path.basename(self.source_path),
            "tags": self.crops,
        }
        if self.page is not None:
            data["page"] = self.page
        with open(self.crops_path, "w") as crops_file:
            yaml.safe_dump(data, crops_file, default_flow_style=False, sort_keys=False)

    def generate(self):
        os.makedirs(self.tag_dir, exist_ok=True)
        os.makedirs(self.mesh_dir, exist_ok=True)

        for crop in self.crops:
            tag_id = crop["id"]
            rect = crop["rect"]
            box = (
                rect["x"],
                rect["y"],
                rect["x"] + rect["width"],
                rect["y"] + rect["height"],
            )
            tag_image_path = os.path.join(self.tag_dir, "tag_%s.png" % tag_id)
            mesh_path = os.path.join(self.mesh_dir, "tag_%s.dae" % tag_id)
            tag_image = self.source_image.crop(box).convert("RGBA")
            tag_image.save(tag_image_path)
            write_dae(mesh_path, tag_image_path)

        self.save_crops()
        messagebox.showinfo(
            "Generated",
            "Generated %d tag image(s) and mesh(es).\n%s\n%s" % (
                len(self.crops),
                self.tag_dir,
                self.mesh_dir,
            ),
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Crop AprilTag images and generate RViz-ready DAE meshes.")
    parser.add_argument("--source", required=True, help="Source image or PDF containing one or more AprilTags.")
    parser.add_argument("--page", type=int, default=1, help="PDF page number to render. Ignored for image sources.")
    parser.add_argument("--dpi", type=int, default=300, help="PDF render DPI.")
    parser.add_argument("--tag-dir", default=default_tag_dir(), help="Directory for generated tag_<id>.png files.")
    parser.add_argument("--mesh-dir", default=default_mesh_dir(), help="Directory for generated tag_<id>.dae files.")
    return parser.parse_args()


def render_pdf_page(source_path, page, dpi):
    if page < 1:
        raise SystemExit("--page must be 1 or greater.")
    if dpi < 30:
        raise SystemExit("--dpi must be 30 or greater.")

    pdftoppm = shutil.which("pdftoppm")
    if not pdftoppm:
        raise SystemExit(
            "PDF source requires pdftoppm from poppler-utils. "
            "Install it with: sudo apt install poppler-utils"
        )

    temp_dir = tempfile.mkdtemp(prefix="apriltag_pdf_source_")
    output_prefix = os.path.join(temp_dir, "page")
    command = [
        pdftoppm,
        "-f",
        str(page),
        "-l",
        str(page),
        "-r",
        str(dpi),
        "-png",
        source_path,
        output_prefix,
    ]
    try:
        subprocess.check_call(command)
    except subprocess.CalledProcessError as error:
        raise SystemExit("failed to render PDF page %d: %s" % (page, error))

    rendered_path = output_prefix + "-%d.png" % page
    if not os.path.exists(rendered_path):
        generated = sorted(
            os.path.join(temp_dir, name)
            for name in os.listdir(temp_dir)
            if name.endswith(".png")
        )
        if not generated:
            raise SystemExit("pdftoppm did not create a PNG for page %d." % page)
        rendered_path = generated[0]
    return rendered_path, [rendered_path, temp_dir]


def resolved_source(args):
    source_path = os.path.abspath(args.source)
    if not os.path.exists(source_path):
        raise SystemExit("source file does not exist: %s" % source_path)

    extension = os.path.splitext(source_path)[1].lower()
    if extension == ".pdf":
        image_path, temp_paths = render_pdf_page(source_path, args.page, args.dpi)
        return source_path, image_path, args.page, temp_paths

    return source_path, source_path, None, []


def main():
    args = parse_args()
    source_path, image_path, page, temp_paths = resolved_source(args)
    root = tk.Tk()
    TagCropApp(root, source_path, image_path, args.tag_dir, args.mesh_dir, page, temp_paths)
    root.mainloop()


if __name__ == "__main__":
    main()
