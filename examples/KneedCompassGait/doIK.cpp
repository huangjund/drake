//
// Created by Junda Huang on 10/30/19.
//
#include <string>

#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace kkkcg {
    using std::string;
    using multibody::InverseKinematics;
    using geometry::SceneGraph;
    using geometry::SourceId;
    using lcm::DrakeLcm;
    using multibody::MultibodyPlant;
    using systems::DiagramBuilder;
    using multibody::Parser;
    int domain() {
//        DiagramBuilder<double> builder;
//        SceneGraph<double>& scenegraph = *builder.AddSystem<SceneGraph>();
//        scenegraph.set_name("Scene_graph");

        string filename = "drake/examples/KneedCompassGait/KneedCompassGait.urdf";
        const string full_name = FindResourceOrThrow(filename);

        MultibodyPlant<double> KCG;
        Parser parser(&KCG);
        parser.AddAllModelsFromFile(full_name);
        KCG.Finalize();

        InverseKinematics kinsol(KCG);
//        auto right_toe = KCG.GetFrameByName("right_lower_leg");
//
//        auto pos1 = kinsol.AddPositionConstraint(right_toe, )
    }
}
}
}

int main() {
    return drake::examples::kkkcg::domain();
}
