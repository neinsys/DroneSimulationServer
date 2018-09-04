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
#include <cmath>
#include <chrono>
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

#include <src/json/json.hpp>

using json = nlohmann::json;

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
void floorCloud(pcl::PointCloud<pcl::PointXYZ>& cloud){
    for(pcl::PointXYZ& p:cloud.points ){
        p.x=floor(p.x);
        p.y=floor(p.y);
        p.z=floor(p.z);
    }
}
void setCenter(pcl::PointCloud<pcl::PointXYZ>& cloud,const float max_width=50.0f){
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
    float fx=(max_width-(Mx-mx))/2.0f;
    float fy=(max_width-(My-my))/2.0f;
    float fz=(max_width-(Mz-mz))/2.0f;

    for(pcl::PointXYZ& p:cloud.points ){
        p.x-=mx;
        p.y-=my;
        p.z-=mz;
        p.x+=fx;
        p.y+=fy;
        p.z+=fz;
    }
}
pcl::PointCloud<pcl::PointXYZ> filteringPointCloud(pcl::PointCloud<pcl::PointXYZ> cloud,const float max_width=50.0f,int target_num=1000,const float leaf_size=1.0f){
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
    float scale = max_width / distance;

    for(pcl::PointXYZ& p:cloud.points ){
        p.x-=mx;
        p.y-=my;
        p.z-=mz;
        p.x*=scale;
        p.y*=scale;
        p.z*=scale;
    }

    pcl::PointCloud<pcl::PointXYZ> voxel_cloud;
    double left = 0.0;
    double right = 1.0;
    bool flag=false;
    for(int _=0;_<500;_++){
        double mid= (left+right)/2.0;

        pcl::PointCloud<pcl::PointXYZ> temp;
        for(pcl::PointXYZ& p:cloud.points ){
            temp.push_back(pcl::PointXYZ(p.x*mid, p.y*mid, p.z*mid));
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_ptr = temp;


        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setInputCloud (cloud_ptr);
        grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        pcl::PointCloud<pcl::PointXYZ>* temp_cloud = new pcl::PointCloud<pcl::PointXYZ>;
        grid.filter (*temp_cloud);
        int sz=temp_cloud->points.size();
        if(sz>=target_num || (!flag &&_==499)){
            flag=true;
            voxel_cloud=*temp_cloud;
        }


        if(sz==target_num)break;
        if(sz>target_num)right=mid;
        else left=mid;
    }
    floorCloud(voxel_cloud);
    setCenter(voxel_cloud,max_width);


    std::random_shuffle(voxel_cloud.points.begin(),voxel_cloud.points.end());
    target_num=std::min<int>(target_num,voxel_cloud.points.size());
    voxel_cloud.points.resize(target_num);
    voxel_cloud.width=target_num;

    std::cout << voxel_cloud << std::endl;

    pcl::io::savePCDFileASCII("debug.pcd",voxel_cloud);

    /*   pcl::PointCloud<pcl::PointXYZ>::Ptr print_cloud(new pcl::PointCloud<pcl::PointXYZ>);
       *print_cloud = *voxel_cloud;
       pcl::visualization::PCLVisualizer vis3 ("VOXELIZED SAMPLES CLOUD");
       vis3.addPointCloud<pcl::PointXYZ> (print_cloud);

       vis3.spin ();
   */
    return voxel_cloud;
}


vector<std::string> obj2PointCloud(const std::string& objContent){
    std::stringstream is(objContent);
    vector<std::string> cloud;
    for(std::string line; std::getline(is, line); )
    {
        std::istringstream in(line);

        std::string v;
        in >> v;
        if (v != "v") continue;

        // Read x y z
        string p;
        std::getline(in,p);
        cloud.push_back(p);
    }
    return cloud;
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
std::tuple<vector<point>,vector<point>,int,int,int,int,int,int> compress_points(vector<point> start,vector<point> end,int optimization){
    auto CX=[](const point& p,const point &q){
        return p.x<q.x;
    };
    auto CY=[](const point& p,const point &q){
        return p.y<q.y;
    };
    auto CZ=[](const point& p,const point &q){
        return p.z<q.z;
    };
    int mx=std::min(std::min_element(start.begin(),start.end(),CX)->x,std::min_element(end.begin(),end.end(),CX)->x);
    int my=std::min(std::min_element(start.begin(),start.end(),CY)->y,std::min_element(end.begin(),end.end(),CY)->y);
    int mz=std::min(std::min_element(start.begin(),start.end(),CZ)->z,std::min_element(end.begin(),end.end(),CZ)->z);
    int Mx=std::max(std::max_element(start.begin(),start.end(),CX)->x,std::max_element(end.begin(),end.end(),CX)->x);
    int My=std::max(std::max_element(start.begin(),start.end(),CY)->y,std::max_element(end.begin(),end.end(),CY)->y);
    int Mz=std::max(std::max_element(start.begin(),start.end(),CZ)->z,std::max_element(end.begin(),end.end(),CZ)->z);
    printf("%d %d %d %d %d %d\n",Mx,My,Mz,mx,my,mz);
    if(optimization<0)return {start,end,Mx,My,Mz,0,0,0};
    int X=Mx-mx+optimization*2;
    int Y=My-my+optimization*2;
    int Z=Mz-mz+optimization*2;
    for(point& p:start){
        p.x+=-mx+optimization;
        p.y+=-my+optimization;
        p.z+=-mz+optimization;
    }
    for(point& p:end){
        p.x+=-mx+optimization;
        p.y+=-my+optimization;
        p.z+=-mz+optimization;
    }
    return {start,end,X,Y,Z,mx-optimization,my-optimization,mz-optimization};

};


vector<vector<point>> objs;
int rest=0;
int max_num=0;

double euclid_dist(point p,point q){
    return hypot(p.x-q.x,hypot(p.y-q.y,p.z-q.z));
}

int main(int argc, char** argv){
    crow::App<> app;
    crow::mustache::set_base("./html");
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
    CROW_ROUTE(app,"/json_test")
            .methods("POST"_method,"GET"_method)
                    ([](const crow::request& req){
                        auto x = crow::json::load(req.body);
                        if(!x)
                            return crow::response(400);


                        return crow::response(200);
                    });
    CROW_ROUTE(app,"/getPointsByObj")
               .methods("POST"_method)
                       ([](const crow::request& req){
                           string content = crow::get_header_value<crow::ci_map >(req.headers,"content-type");
                           string content_type = content.substr(0,content.find(';'));
                           std::transform(content_type.begin(), content_type.end(), content_type.begin(), ::tolower);

                           size_t idx = content.find("boundary=")+string("boundary=").length();
                           string boundary = "--"+content.substr(idx);

                           vector<pair<string,string>> objFiles = readFileFormData(req.body,boundary);
                           json objs = json::array();
                           for(const pair<string,string>& obj:objFiles){
                               json j={};
                               j["filename"]=obj.first;
                               vector<string> pc = obj2PointCloud(obj.second);
                               json points=json::array();
                               for(string p : pc){
                                   points.push_back(p);
                               }
                               j["points"]=points;
                               objs.push_back(j);
                           }
                           return objs.dump();
                       });
    CROW_ROUTE(app,"/filteringPoints")
               .methods("POST"_method)
                       ([](const crow::request& req){
                           auto param = json::parse(req.body);


                           int num = param["number"];
                           float max_width = 50.0f;
                           float leaf_size = 1.0f;
                           if(param.count("width")){
                               max_width = param["width"];
                           }
                           if(param.count("leaf_size")){
                               leaf_size=param["leaf_size"];
                           }
                           auto objs = param["objs"];
                           std::cout<<"number : " << num << ", width : " << max_width << ", leaf size : " << leaf_size <<std::endl;
                           std::cout << objs.size();
                           json res=json::array();

                           for(int i=0;i<objs.size();i++){
                               auto& obj=objs[i];
                               pcl::PointCloud <pcl::PointXYZ> cloud;

                               for (const string& line:obj["points"]) {
                                   std::istringstream in(line);

                                   // Read x y z
                                   float x, y, z;
                                   in >> x >> y >> z;
                                   cloud.push_back(pcl::PointXYZ(x, y, z));
                               }
                               //auto filter_cloud =test(cloud);
                               auto filter_cloud = filteringPointCloud(cloud,max_width,num,leaf_size);
                               json j={};
                               j["filename"]=obj["filename"];
                               j["max_width"]=max_width;
                               j["leaf_size"] = leaf_size;
                               json points=json::array();
                                std::cout<<filter_cloud.points.size()<<std::endl;
                                for(int j=0;j<filter_cloud.points.size();j++){
                                    const auto& p = filter_cloud.points[j];
                                    json point = {(int)p.x,(int)p.y,(int)p.z};
                                    points.push_back(point);
                                }
                                j["points"]=points;
                                res.push_back(j);
                           }
                           return res.dump();
                       });
    CROW_ROUTE(app,"/calculatePath")
            .methods("POST"_method)
                    ([](const crow::request& req){
                        auto param = json::parse(req.body);
                        vector<vector<point>> objs;
                        for(const auto& obj:param["objects"]){
                            vector<point> points;
                            for(const auto& p:obj["points"]){
                                points.push_back({p[0],p[1],p[2]});
                            }
                            objs.push_back(points);
                        }
                        string algorithm = param["algorithm"];
                        int rest = param["rest"];
                        int optimization=-1;
                        if(param.count("optimization")){
                            optimization = param["optimization"];
                        }
                        std::vector<analysis> paths((int)objs.size()-1);
                        vector<long> clocks((int)objs.size()-1);
                        using namespace std::chrono;
#pragma omp parallel for
                        for(int i=0;i<(int)objs.size()-1;i++){
                            auto [start,end,X,Y,Z,mx,my,mz] = compress_points(objs[i],objs[i+1],optimization);

                            system_clock::time_point start_t = system_clock::now();

                            if(algorithm=="Dinic")
                                paths[i] = find_path_using_dinic(start,end,X+1,Y+1,Z+1);
                            else if(algorithm == "MCMF")
                                paths[i]=find_path_using_mcmf(start,end,X+1,Y+1,Z+1);
                            else if(algorithm=="DinicAndMCMF")
                                paths[i]=find_path_using_mcmf_and_dinic(start,end,X+1,Y+1,Z+1);
                            system_clock::time_point end_t = system_clock::now();
                            milliseconds calc = duration_cast<milliseconds>(end_t-start_t);
                            clocks[i]=calc.count();

                            for(path* p:paths[i].paths){
                                for(node* it=p->head;it!=NULL;it=it->next){
                                    it->p.x+=mx;
                                    it->p.y+=my;
                                    it->p.z+=mz;
                                }
                            }
                        }
                        json ret;
                        ret["analysis"]=json::array();
                        for(int i=0;i<paths.size();i++){
                            auto& Path = paths[i].paths;
                            auto collisions = paths[i].collsions;
                            auto& calcTime=clocks[i];
                            auto& T_calc_time = paths[i].T_calc_time;
                            auto& P_calc_time = paths[i].P_calc_time;
                            json analysis;
                            double max_detour=0.0;
                            double sum_euclid=0.0;
                            double sum_path_length=0.0;
                            double sum_detour=0.0;
                            int T=0;
                            for(path* p:Path){
                                double euclid=euclid_dist(p->head->p,p->tail->p);
                                double path_length=0.0;
                                for(node* it=p->head;it->next!=NULL;it=it->next){
                                    path_length+=euclid_dist(it->p,it->next->p);
                                }

                                double detour = path_length / euclid;
                                if(fabs(euclid)<1e-9)detour=1.0;
                                max_detour=std::max(max_detour,detour);
                                sum_euclid+=euclid;
                                sum_path_length+=path_length;
                                sum_detour+=detour;
                                T=p->size();
                            }
                            printf("%ld\n",calcTime);
                            analysis["compress_value"] = optimization;
                            analysis["calcTime"]=calcTime;
                            analysis["T_calcTime"]=T_calc_time;
                            analysis["P_calcTime"]=P_calc_time;
                            analysis["time"] = T;
                            analysis["max_detour"] = max_detour;
                            analysis["avg_detour(sum path length / sum euclidean distance)"] = sum_path_length / sum_euclid;
                            analysis["avg_detour(sum detour / number of path)"] = sum_detour / (int)Path.size();
                            analysis["sum_path_length"] = sum_path_length;
                            analysis["algorithm"] = algorithm;
                            analysis["collisions"] = collisions;
                            analysis["Mx"]=paths[i].Mx;
                            analysis["My"]=paths[i].My;
                            analysis["Mz"]=paths[i].Mz;
                            std::ostringstream image;
                            image << param["objects"][i]["filename"];
                            image << "(";
                            image << (int)param["objects"][i]["points"].size();
                            image << ") -> ";
                            image << param["objects"][i+1]["filename"];
                            image << "(";
                            image << (int)param["objects"][i+1]["points"].size();
                            image << ")";
                            analysis["image"] = image.str();
                            ret["analysis"].push_back(analysis);
                        }
                        printf("???\n");
                        std::vector<path*> new_path = merge_path(paths,rest);

                        printf("???\n");
                        paths.clear();
                        json total_path=json::array();
                        if(new_path.empty()){
                            for(auto& obj : objs){
                                for(const point& p:obj) {

                                    json one_path=json::array();
                                    json point ={p.x,p.y,p.z};
                                    one_path.push_back(point);
                                    total_path.push_back(one_path);
                                }
                            }

                        }
                        else{
                            printf("???\n");
                            for(const path* P:new_path){
                                json one_path=json::array();
                                for(auto it=P->head;it!=NULL;it=it->next){
                                    point p=it->p;
                                    json point ={p.x,p.y,p.z};
                                    one_path.push_back(point);
                                }
                                total_path.push_back(one_path);
                            }
                            for(path* P:new_path){
                                delete P;
                            }
                        }
                        ret["paths"]=total_path;
                        printf("???\n");
                        return ret.dump();
                    });
    app.port(8080).multithreaded().run();
    return 0;

}
