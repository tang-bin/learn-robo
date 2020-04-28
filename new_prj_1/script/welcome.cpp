#include <gazebo/gazebo.hh>

namespace gazebo
{
    class WelcomeToMaze : public WorldPlugin
    {
        public: WelcomeToMaze() : WorldPlugin()
        {
            printf("Welcome to Bin's Maze world.\n");
        }

        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {

        }
    };

    GZ_REGISTER_WORLD_PLUGIN(WelcomeToMaze)
}