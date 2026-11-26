// Local Gazebo Classic console-log redirection. No HOME or system file changes.
// FileLogger has no environment option for its console directory in Gazebo 11.
// Interpose only Init; all ordinary Gazebo logging and physics remain unchanged.
#include <gazebo/common/Console.hh>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

void gazebo::common::FileLogger::Init(const std::string &prefix,
                                      const std::string &filename) {
  const char *directory = std::getenv("AGRI_GAZEBO_CONSOLE_DIR");
  if (!directory || !*directory)
    throw std::runtime_error("AGRI_GAZEBO_CONSOLE_DIR must be set");
  auto folder = std::filesystem::path(directory) / prefix;
  std::filesystem::create_directories(folder);
  auto *buffer = static_cast<Buffer *>(this->rdbuf());
  if (buffer->stream) {
    buffer->stream->close();
    delete buffer->stream;
  }
  buffer->stream = new std::ofstream(folder / std::filesystem::path(filename).filename());
  if (!*buffer->stream) throw std::runtime_error("Cannot create local Gazebo log");
  logDirectory = folder.string();
}
