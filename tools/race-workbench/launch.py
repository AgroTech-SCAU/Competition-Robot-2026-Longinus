"""Serve the offline Longinus workbench on this computer."""
from argparse import ArgumentParser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import webbrowser


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8768, help="Local port for the browser workspace")
    parser.add_argument("--no-open", action="store_true", help="Print the URL without opening a browser")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    directory = Path(__file__).resolve().parent
    handler = partial(SimpleHTTPRequestHandler, directory=str(directory))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    except OSError as error:
        parser.exit(1, f"Cannot start Longinus on localhost port {args.port}: {error}\n")
    with server:
        url = f"http://127.0.0.1:{server.server_port}/"
        print(f"Longinus: {url}\nKeep this window open. Press Ctrl+C to stop.", flush=True)
        if not args.no_open:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
