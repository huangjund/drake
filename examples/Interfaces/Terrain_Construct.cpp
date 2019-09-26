#include "drake/examples/Terrain_Construct.h"

namespace drake {
namespace examples {
namespace compass_gait {
    void AddTerrainToWorld(RigidBodyTree<double>* tree, std::string kFilePath,
                           double x_bias, double y_bias, double z_bias) {
            DrakeShapes::Mesh geom("scenario", drake::FindResourceOrThrow(kFilePath));

            Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
            Eigen::AngleAxisd rotation_base(M_PI/2, Vector3<double>(1, 0, 0));
            X_WB.rotate(rotation_base);
            X_WB.translation() << x_bias, y_bias, z_bias;

            // Defines a color called "desert sand"
            Eigen::Vector4d color(0.9297, 0.7930, 0.6758, 1);

            RigidBody<double>& world = tree->world();
            world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color));
            tree->addCollisionElement(
                    drake::multibody::collision::Element(geom, X_WB, &world), world,
                    "terrain");
            tree->compile();
    }
}
}  // examples
}  // drake