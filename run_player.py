import argparse
import logging
import os
import sys

from mujococodebase.agent import Agent


def _configure_logging(level: int) -> None:
    """
    Configure the root logger once. Library modules must not call basicConfig:
    import order can leave DEBUG enabled and flood stderr (slowing the agent).
    """
    ch = logging.StreamHandler()
    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s (%(filename)s:%(lineno)d) - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    ch.setFormatter(formatter)
    ch.setLevel(level)
    kwargs = {"handlers": [ch], "level": level}
    if sys.version_info >= (3, 8):
        kwargs["force"] = True
    logging.basicConfig(**kwargs)


parser = argparse.ArgumentParser(description="Start a MuJoCo Agent.")

parser.add_argument("-t", "--team", type=str, default="Default", help="Team name")
parser.add_argument("-n", "--number", type=int, default=1, help="Player number")
parser.add_argument("--host", type=str, default="127.0.0.1", help="Server host")
parser.add_argument("--port", type=int, default=60000, help="Server port")
parser.add_argument("-f", "--field", type=str, default="hl_adult", help="Field to be played")

parser.add_argument(
    "--spawn-x",
    type=float,
    default=None,
    help="Optional custom initial X position (field coordinates) for this player.",
)
parser.add_argument(
    "--spawn-y",
    type=float,
    default=None,
    help="Optional custom initial Y position (field coordinates) for this player.",
)
parser.add_argument(
    "--spawn-rot",
    type=float,
    default=None,
    help="Optional custom initial rotation (degrees) for this player.",
)
parser.add_argument(
    "-v",
    "--verbose",
    action="store_true",
    help="Enable DEBUG logging (very noisy; slows the agent loop).",
)


def main() -> None:
    args = parser.parse_args()
    _configure_logging(logging.DEBUG if args.verbose else logging.INFO)

    # Optional single-player custom spawn position.
    if args.spawn_x is not None and args.spawn_y is not None and args.spawn_rot is not None:
        os.environ["PLAYER_SPAWN_X"] = str(args.spawn_x)
        os.environ["PLAYER_SPAWN_Y"] = str(args.spawn_y)
        os.environ["PLAYER_SPAWN_ROT"] = str(args.spawn_rot)

    player = Agent(
        team_name=args.team,
        number=args.number,
        host=args.host,
        port=args.port,
        field=args.field,
    )

    player.run()


if __name__ == "__main__":
    main()
