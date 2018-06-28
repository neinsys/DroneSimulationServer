#include "crow.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <locale>
#include <limits.h>
#include <algorithm>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include "flowgraph.h"
#include "find_path.h"
#include "converter.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
using std::string;
using std::ofstream;
using std::endl;
using std::vector;
using std::pair;
const int cm=100;
const float leaf_size=1.0f;
const std::string filePath = "PointCloudFile";
namespace fs = boost::filesystem;
vector<pair<string,string>> readFileFormData(const string& body,const string& boundary){
    std::vector<pair<string,string>> objFiles;
    size_t index = 0;
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


        size_t file_start = body.find("\r\n\r\n",next)+4;
        size_t file_end = body.find("\r\n"+boundary,file_start);
        string file_content = body.substr(file_start,file_end-file_start);
        objFiles.push_back({filename,file_content});
        index=next+boundary.length();
    }
    return objFiles;
}

pcl::PointCloud<pcl::PointXYZ> convertPointCloud(const string& content,const float width=50.0f,int num=1000){
    pcl::PointCloud<pcl::PointXYZ> cloud=Converter().obj2PointCloud(content);

    float mx=1e18f;
    float Mx=-1e18f;
    float my=1e18f;
    float My=-1e18f;
    float mz=1e18f;
    float Mz=-1e18f;
    std::cout<<cloud<<std::endl<<cloud.points.size() << std::endl;
    for(const pcl::PointXYZ& p:cloud.points ){
        mx=std::min(mx,p.x);
        Mx=std::max(Mx,p.x);
        my=std::min(my,p.y);
        My=std::max(My,p.y);
        mz=std::min(mz,p.z);
        Mz=std::max(Mz,p.z);
    }
    std::cout << Mx-mx << ' ' << My-my <<' '<< Mz - mz<< std::endl;
    float distance = std::max({Mx-mx,My-my,Mz-mz});
    float scale = width / distance;

    for(pcl::PointXYZ& p:cloud.points ){
        p.x-=mx;
        p.y-=my;
        p.z-=mz;
        p.x*=scale;
        p.y*=scale;
        p.z*=scale;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_ptr = cloud;
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud (cloud_ptr);
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<pcl::PointXYZ>* voxel_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    grid.filter (*voxel_cloud);

    std::cout << *voxel_cloud << std::endl;

    std::random_shuffle(voxel_cloud->points.begin(),voxel_cloud->points.end());
    num=std::min<int>(num,voxel_cloud->points.size());
    voxel_cloud->points.resize(num);
    voxel_cloud->width=num;

    std::cout << *voxel_cloud << std::endl;

    pcl::io::savePCDFileASCII("debug.pcd",*voxel_cloud);

 /*   pcl::PointCloud<pcl::PointXYZ>::Ptr print_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *print_cloud = *voxel_cloud;
    pcl::visualization::PCLVisualizer vis3 ("VOXELIZED SAMPLES CLOUD");
    vis3.addPointCloud<pcl::PointXYZ> (print_cloud);

    vis3.spin ();
*/
    return *voxel_cloud;
}

void savePointCloud(const string& filename,const pcl::PointCloud<pcl::PointXYZ>& pc){
    std::ofstream outFile(filePath+"/"+filename);
    outFile << pc.points.size()<<std::endl;
    for(const auto& p:pc.points){
        outFile << int(p.x) << ' ' << int(p.y) << ' ' << int(p.z) << std::endl;
    }
    outFile.close();
}
vector<point> loadPointCloud(const string& filename){
    vector<point> pc;
    std::ifstream inFile(filePath+"/"+filename);
    int num;
    inFile >>num;

    for(int i=0;i<num;i++){
        int x,y,z;
        inFile >> x>>y>>z;
        pc.push_back({x,y,z});
    }
    inFile.close();
    return pc;
}

vector<vector<point>> objs;
int rest=0;
int max_num=0;
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

    CROW_ROUTE(app,"/findPath")
            .methods("GET"_method)
                    ([&](const crow::request& req){
                        int num = boost::lexical_cast<int>(req.url_params.get("num"));
                        std::vector<std::vector<path*>> paths;
                        vector<vector<point>> objs;
                        int rest=0;
                        if(req.url_params.get("rest")!=nullptr){
                            rest=boost::lexical_cast<int>(req.url_params.get("rest"));
                        }
                        for(int i=1;i<=num;i++){
                            char key[40];
                            sprintf(key,"input%d",i);
                            const std::string filename = req.url_params.get(key);
                            objs.push_back(loadPointCloud(filename));
                        }

                        int max_num=0;
                        for(auto& obj:objs){
                            max_num=std::max<int>(max_num,obj.size());
                        }
                        const int N=50;
                        for(auto& obj:objs){
                            for(point& p:obj){
                                p.z++;
                            }
                            int add=max_num-obj.size();
                            for(int i=0;i<add;i++){
                                int x=i/N;
                                int y=i%N;
                                int z=0;
                                obj.push_back({x,y,z});
                            }
                        }



                        for(int i=0;i<objs.size()-1;i++){
                            auto start = objs[i];
                            auto end = objs[i+1];
                            int X=0,Y=0,Z=0;
                            for(const point& p:start){
                                X=std::max(X,p.x);
                                Y=std::max(Y,p.y);
                                Z=std::max(Z,p.z);
                            }
                            for(const point& p:end){
                                X=std::max(X,p.x);
                                Y=std::max(Y,p.y);
                                Z=std::max(Z,p.z);
                            }

                            auto path = find_path(start,end,X+1,Y+1,Z+1);
                            paths.push_back(path);
                        }
                        std::cout << "aa" << std::endl;
                        std::vector<path*> new_path = merge_path(paths,rest);
                        std::cout << "bb" << std::endl;
                        paths.clear();

                        std::stringstream s;
                        if(new_path.empty()){
                            s<<max_num<< ' '<< 0 << '\n';
                            for(auto& obj : objs){
                                for(const point& p:obj) {
                                    s << p.x * cm << ' ' << p.y * cm << ' ' << p.z * cm << '\n';
                                }
                            }
                            return s.str();
                        }
                        int n=new_path.size();
                        int t=new_path.back()->size()-1;

                        s << n << ' ' << t<<'\n';
                        for(const path* P:new_path){
                            for(auto it=P->head;it!=NULL;it=it->next){
                                point p=it->p;
                                s<<p.x*cm <<' ' << p.y*cm<<' '<<p.z*cm<<'\n';
                            }
                        }

                        return s.str();
                    });

    CROW_ROUTE(app,"/findPath2")
            .methods("GET"_method)
                    ([&objs,&rest](const crow::request& req){
                        auto images = req.url_params.get_list("image");

                        rest=0;
                        objs.clear();
                        if(req.url_params.get("rest")!=nullptr){
                            rest=boost::lexical_cast<int>(req.url_params.get("rest"));
                        }
                        for(auto image : images){
                            const std::string filename = image;
                            objs.push_back(loadPointCloud(filename));
                        }

                        max_num=0;
                        for(auto& obj:objs){
                            max_num=std::max<int>(max_num,obj.size());
                        }
                        const int N=50;
                        for(auto& obj:objs){
                            for(point& p:obj){
                                p.z++;
                            }
                            int add=max_num-obj.size();
                            for(int i=0;i<add;i++){
                                int x=i/N;
                                int y=i%N;
                                int z=0;
                                obj.push_back({x,y,z});
                            }
                        }
                        return "OK";
                    });

    CROW_ROUTE(app,"/findPath3")
            .methods("GET"_method)
                    ([&](const crow::request& req){
                        crow::mustache::context ctx;
                        return crow::mustache::load("example.txt").render();
                    });
    CROW_ROUTE(app,"/findPath4")
            .methods("GET"_method)
                    ([&](const crow::request& req){

                        std::vector<std::vector<path*>> paths;

                        for(int i=0;i<(int)objs.size()-1;i++){
                            auto start = objs[i];
                            auto end = objs[i+1];
                            int X=0,Y=0,Z=0;
                            for(const point& p:start){
                                X=std::max(X,p.x);
                                Y=std::max(Y,p.y);
                                Z=std::max(Z,p.z);
                            }
                            for(const point& p:end){
                                X=std::max(X,p.x);
                                Y=std::max(Y,p.y);
                                Z=std::max(Z,p.z);
                            }

                            auto path = find_path(start,end,X+1,Y+1,Z+1);
                            paths.push_back(path);
                        }
                        std::cout << "aa" << std::endl;
                        std::vector<path*> new_path = merge_path(paths,rest);
                        std::cout << "bb" << std::endl;
                        paths.clear();
                        std::stringstream s;
                        if(new_path.empty()){
                            s<<max_num<< ' '<< 0 << '\n';
                            for(auto& obj : objs){
                                for(const point& p:obj) {
                                    s << p.x * cm << ' ' << p.y * cm << ' ' << p.z * cm << '\n';
                                }
                            }
                            return s.str();
                        }
                        int n=new_path.size();
                        int t=new_path.back()->size()-1;

                        s << n << ' ' << t<<'\n';
                        for(const path* P:new_path){
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
                    ([&filePath](const crow::request& req){
                        string content = crow::get_header_value<crow::ci_map >(req.headers,"content-type");
                        string content_type = content.substr(0,content.find(';'));
                        std::transform(content_type.begin(), content_type.end(), content_type.begin(), ::tolower);
                        if(content_type != "multipart/form-data"){
                            return "Error";
                        }
                        size_t idx = content.find("boundary=")+string("boundary=").length();
                        string boundary = "--"+content.substr(idx);

                        vector<pair<string,string>> objFiles = readFileFormData(req.body,boundary);

                        std::cout<<objFiles.size() <<std::endl;

                        fs::create_directory(filePath);

                        for(const pair<string,string>& obj:objFiles){
                            auto pc = convertPointCloud(obj.second);
                            savePointCloud(obj.first,pc);
                        }

                        return "OK";
                    });

    CROW_ROUTE(app,"/imageList")
            .methods("POST"_method,"GET"_method)
                    ([&filePath](const crow::request& req){
                        std::stringstream list;
                        list <<"<!DOCTYPE html>\n"
                               "<html lang=\"en\">\n"
                               "<head>\n"
                               "    <meta charset=\"UTF-8\">\n"
                               "    <title>Image List</title>\n"
                               "</head>\n"
                               "<body>\n"
                               "<form method=\"get\" action=\"findPath2\">";
                        for(auto&& file : fs::recursive_directory_iterator(filePath)){
                            std::string filename=file.path().leaf().generic_string();
                            list<<"<input type=\"checkbox\" name=\"image[]\" value=\""<<filename<<"\">" <<filename<<"<br>";
                        }
                        list <<"<input type=\"text\" name=\"rest\" value=\"0\">이미지간 간격<br>"
                               "<input type=\"submit\" value=\"전송\">\n"
                               "</form>\n"
                               "</body>\n"
                               "</html>";
                        return list.str();
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

        vector<pair<string,string>> objFiles = readFileFormData(req.body,boundary);

        for(const pair<string,string>& obj:objFiles){
            pcl::PointCloud<pcl::PointXYZ> pc = Converter().obj2PointCloud(obj.second);
            std::cout<<pc<<std::endl;
        }

        return "Hello world";
    });
    app.port(8080).multithreaded().run();
    return 0;

}
