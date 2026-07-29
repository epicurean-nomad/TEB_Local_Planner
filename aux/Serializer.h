#pragma once

#include <string>

class AreaGraph;
struct BufWriter;
struct BufReader;

class Serializer {
public:
    static bool loadArea      (AreaGraph& graph, const std::string& filepath, std::string& error);
    static bool saveAvRuntime (const AreaGraph& graph, const std::string& filepath, std::string& error);
    static bool loadAvRuntime (AreaGraph& graph, const std::string& filepath, std::string& error);

private:
    static void writeAreaGraph      (BufWriter& w, const AreaGraph& g);
    static void writeRuntimeSections(BufWriter& w, const AreaGraph& g);
    static bool readRuntimeSections (BufReader& r, AreaGraph& g, std::string& error);
};
