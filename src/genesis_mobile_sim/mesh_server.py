"""Serve only the existing Go2 model assets through Robo-Boy's mesh proxy."""

import os
import threading
from contextlib import contextmanager
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .robot_model import model_path


class MeshHandler(SimpleHTTPRequestHandler):
    def send_head(self):
        path = Path(self.translate_path(self.path)).resolve()
        root = model_path().parents[1].resolve()
        if (
            not path.is_relative_to(root)
            or not path.is_file()
            or path.suffix.lower() not in {".dae", ".stl", ".obj", ".mtl", ".png", ".jpg", ".jpeg"}
        ):
            self.send_error(404, "Go2 model asset not found")
            return None
        return super().send_head()

    def log_message(self, format, *args):
        # Mesh requests are bounded read-only files; lifecycle and failures use ROS logs.
        pass


@contextmanager
def serve_meshes(host="0.0.0.0", port=None):
    if port is None:
        port = int(os.environ.get("MESH_RESOURCES_PORT", "8000"))
    handler = partial(MeshHandler, directory=str(model_path().parents[2]))
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
