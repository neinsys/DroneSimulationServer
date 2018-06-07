#include "crow.h"
#include <iostream>
#include <fstream>
#include <string>
#include "flowgraph.h"
#include "find_path.h"
using std::string;
using std::ofstream;
using std::endl;




int main(int argc, char** argv){
    crow::SimpleApp app;
    CROW_ROUTE(app,"/")
    ([]{
        return "Hello world";
    });
    app.port(8080).run();
    return 0;

}
