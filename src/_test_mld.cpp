#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

#include <icecream.hpp>

using std::string;

using namespace gazebo;

class MultiLidars : public ServerFixture
{
public:

    MultiLidars();    
    
    void Run();
    
    void TestBody() override {}
};

MultiLidars::MultiLidars()
{
    
}

void MultiLidars::Run()
{
    string world_fn = "";
#ifdef ROOT_DIR
    world_fn = ROOT_DIR;
#endif

    world_fn += "/worlds/trees";

    IC(world_fn);
    
    // Load the shapes world
    Load(world_fn, true, "ode");
    physics::WorldPtr world = physics::get_world("default");

    gazebo::physics::RayShapePtr ray =
    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        world->Physics()->CreateShape("ray",
            gazebo::physics::CollisionPtr()));

    double dist;
    std::string entity;

    ray->SetPoints(ignition::math::Vector3d(0, 0, 0.5),
                    ignition::math::Vector3d(10, 0, 0.5));
    ray->GetIntersection(dist, entity);

    IC(dist, 0.5, entity);

    // EXPECT_NEAR(dist, 0.5, 1e-4);
    // EXPECT_EQ(entity, "box::link::collision");

    ray->SetPoints(ignition::math::Vector3d(-1, 0, 0.5),
                    ignition::math::Vector3d(50, 0, 0.5));
    ray->GetIntersection(dist, entity);

    IC(dist, 0.5, entity);

    // EXPECT_NEAR(dist, 0.5, 1e-4);
    // EXPECT_EQ(entity, "sphere::link::collision");

    ray->SetPoints(ignition::math::Vector3d(-1, -1.5, 0.5),
                    ignition::math::Vector3d(10, -1.5, 0.5));
    ray->GetIntersection(dist, entity);

    IC(dist, 0.5, entity);
    // EXPECT_NEAR(dist, 0.5, 1e-4);
    // EXPECT_EQ(entity, "cylinder::link::collision");

    ray->SetPoints(ignition::math::Vector3d(-1, -10.5, 0.5),
                    ignition::math::Vector3d(10, -10.5, 0.5));
    ray->GetIntersection(dist, entity);

    IC(dist, 1000, entity);
    // EXPECT_NEAR(dist, 1000, 1e-4);
    // EXPECT_TRUE(entity.empty());
}

int main(int argc, char **argv)
{

    MultiLidars mld;
    mld.Run();

    IC("finished!");

    return 0;
}
