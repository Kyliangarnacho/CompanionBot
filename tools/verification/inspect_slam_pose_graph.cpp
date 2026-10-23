// Offline inspection through the installed, unmodified upstream Karto API.
// No ROS node, controller, scan matcher or custom serialization implementation.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <karto_sdk/Mapper.h>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  try {
    karto::Dataset dataset;
    // Mapper references sensors registered by Dataset; destroy Mapper first.
    karto::Mapper mapper;
    mapper.LoadFromFile(std::string(argv[1]) + ".posegraph");
    dataset.LoadFromFile(std::string(argv[1]) + ".data");
    // Same registration step as SlamToolbox::loadSerializedPoseGraph().
    for (const auto object : dataset.GetLasers()) {
      auto sensor = dynamic_cast<karto::Sensor *>(object);
      if (!sensor) return 1;
      karto::SensorManager::GetInstance()->RegisterSensor(sensor);
    }
    std::ofstream out(argv[2]);
    out << std::setprecision(17) << "{\"scans\":[";
    bool first = true;
    for (const auto scan : mapper.GetAllProcessedScans()) {
      if (!first) out << ',';
      first = false;
      const auto pose = scan->GetCorrectedPose();
      out << "{\"id\":" << scan->GetUniqueId() << ",\"time_s\":" << scan->GetTime()
          << ",\"pose\":[" << pose.GetX() << ',' << pose.GetY() << ',' << pose.GetHeading() << "]}";
    }
    out << "],\"edges\":[";
    first = true;
    for (const auto edge : mapper.GetGraph()->GetEdges()) {
      if (!first) out << ',';
      first = false;
      const auto a = edge->GetSource()->GetObject();
      const auto b = edge->GetTarget()->GetObject();
      out << "{\"source\":" << a->GetUniqueId() << ",\"target\":" << b->GetUniqueId()
          << ",\"source_time_s\":" << a->GetTime() << ",\"target_time_s\":" << b->GetTime() << '}';
    }
    out << "]}\n";
    return out.good() ? 0 : 1;
  } catch (const karto::Exception &error) {
    std::cerr << error.GetErrorMessage() << '\n';
    return 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
