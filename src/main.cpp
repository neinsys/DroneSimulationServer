#include "crow.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <locale>
#include "flowgraph.h"
#include "find_path.h"
#include "converter.h"
using std::string;
using std::ofstream;
using std::endl;
const int cm=100;

int main(int argc, char** argv){
    crow::App<> app;
    crow::mustache::set_base("./html");
    CROW_ROUTE(app,"/example")
       .methods("GET"_method,"POST"_method)
   ([](const crow::request& req){
       std::vector<point> start,end;
        for(int i=0;i<10;i++){
            for(int j=0;j<10;j++){
                for(int k=0;k<10;k++){
                    start.push_back({i,j,k});
                    end.push_back({i,j,k+40});
                }
            }
        }
        auto paths = find_path(start,end,10,10,60);
        int n=paths.size();
        int t=paths.back()->size()-1;
        std::stringstream s;
        s << n << ' ' << t<<'\n';
        for(const path* P:paths){
            for(auto it=P->head;it!=NULL;it=it->next){
                point p=it->p;
                s<<p.x*cm <<' ' << p.y*cm<<' '<<p.z*cm<<'\n';
            }
        }

       return s.str();
   });

    CROW_ROUTE(app,"/insertImage")
            .methods("GET"_method)
                    ([](const crow::request& req){
                        crow::mustache::context ctx;
                        return crow::mustache::load("insertImage.html").render();
                    });

    CROW_ROUTE(app,"/insertImage")
            .methods("POST"_method)
                    ([](const crow::request& req){
                        string content = crow::get_header_value<crow::ci_map >(req.headers,"content-type");
                        string content_type = content.substr(0,content.find(';'));
                        std::transform(content_type.begin(), content_type.end(), content_type.begin(), ::tolower);
                        std::cout << content_type<<std::endl << req.body <<std::endl;
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

                            size_t name_start = body.find("name=",next)+6;
                            size_t name_end = body.find("\";",name_start);
                            string name = body.substr(name_start,name_end-name_start);

                            size_t filename_start = body.find("filename=",next)+10;
                            size_t filename_end = body.find("\"",filename_start);
                            string filename = body.substr(filename_start,filename_end-filename_start);

                            std::cout <<name <<std::endl<<filename<<std::endl;

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

            size_t name_start = body.find("name=",next)+6;
            size_t name_end = body.find("\";",name_start);
            string name = body.substr(name_start,name_end-name_start);

            size_t filename_start = body.find("filename=",next)+10;
            size_t filename_end = body.find("\"",filename_start);
            string filename = body.substr(filename_start,filename_end-filename_start);

            std::cout <<name <<std::endl<<filename<<std::endl;

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
