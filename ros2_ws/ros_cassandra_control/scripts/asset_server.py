#!/usr/bin/env python3
"""ROS 2 node serving Cassandra assets to remote Foxglove clients."""

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


class AssetRequestHandler(SimpleHTTPRequestHandler):
    RESPONSE_HEADERS = {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET, HEAD, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
        "Cache-Control": "public, max-age=3600",
    }

    def end_headers(self):
        for name, value in self.RESPONSE_HEADERS.items():
            self.send_header(name, value)
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()


class AssetServer(Node):
    def __init__(self):
        super().__init__(
            "cassandra_asset_server",
            automatically_declare_parameters_from_overrides=True,
        )

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        description_path = str(self.get_parameter("description_path").value).strip()
        if not description_path:
            description_path = get_package_share_directory(
                "ros_cassandra_description"
            )

        handler = partial(AssetRequestHandler, directory=description_path)
        self._server = ThreadingHTTPServer((host, port), handler)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(
            f"Cassandra assets available at http://{host}:{port}/meshes/"
        )

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AssetServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
