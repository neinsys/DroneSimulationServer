#include "crow.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <locale>
#include "flowgraph.h"
#include "find_path.h"
#include "converter.h"
using std::string;
using std::ofstream;
using std::endl;

int main(int argc, char** argv){
    crow::App<> app;
    CROW_ROUTE(app,"/")
        .methods("POST"_method)
    ([](const crow::request& req){
        string content = crow::get_header_value<crow::ci_map >(req.headers,"content-type");
        string content_type = content.substr(0,content.find(';'));
        std::transform(content_type.begin(), content_type.end(), content_type.begin(), ::tolower);
        if(content_type != "multipart/form-data"){
            return "Error";
        }
        size_t idx = content.find("boundary=")+string("boundary=").length();
        string boundary = "--"+content.substr(idx);

        size_t index = 0;
        const string& body = req.body;
        std::vector<string> objFiles;
        while(true){
            size_t next = body.find(boundary,index);
            size_t finish = body.find(boundary+"--");
            if(next == finish)break;

            size_t file_start = body.find("\r\n\r\n",next)+4;
            size_t file_end = body.find("\r\n"+boundary,file_start);
            string file_content = body.substr(file_start,file_end-file_start);
            objFiles.push_back(file_content);
            index=next+boundary.length();
        }

        for(const string& obj:objFiles){
            pcl::PointCloud<pcl::PointXYZ> pc = Converter().obj2PointCloud(obj);
            std::cout<<pc<<std::endl;
        }

        return "Hello world";
    });
    app.port(8080).run();
    return 0;

}
