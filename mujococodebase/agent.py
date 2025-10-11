import logging

from mujococodebase.decision_maker import DecisionMaker
from mujococodebase.robot import T1, Robot
from mujococodebase.skills.skills_manager import SkillsManager
from mujococodebase.world.world import World
from mujococodebase.server import Server
from mujococodebase.world_parser import WorldParser

logger = logging.getLogger(__file__)


class Agent:
    def __init__(
        self,
        team_name: str = "Default",
        number: int = 1,
        host: str = "localhost",
        port: int = 60000,
        field: str = 'fifa'
    ):
        self.world: World = World(agent=self, team_name=team_name, number=number, field_name=field)
        self.world_parser: WorldParser = WorldParser(agent=self)
        self.server: Server = Server(
            host=host, port=port, world_parser=self.world_parser
        )
        self.robot: Robot = T1(agent=self)
        self.skills_manager: SkillsManager = SkillsManager(agent=self)
        self.decision_maker: DecisionMaker = DecisionMaker(agent=self)

    def run(self):
        """
        Run the simulation client.
        """
        self.server.connect()

        self.server.send_immediate(
            f"(init {self.robot.name} {self.world.team_name} {self.world.number})"
        )

        while True:
            try:
                self.server.receive()

                self.world.update()

                self.decision_maker.update_current_behavior()

                self.server.send()
            except Exception:
                raise

        # self.shutdown()

    def shutdown(self):
        logger.info("Shutting down.")
        self.server.shutdown()

    def think(self):
        pass
