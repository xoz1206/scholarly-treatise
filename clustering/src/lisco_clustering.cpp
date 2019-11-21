#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include<vector>
#include<algorithm>
#include<cstdlib>
#include<cmath>
#include<ctime>
#include"clustering/lisco_clustering.h"
#include<visualization_msgs/MarkerArray.h>
//#include<visualization_msgs/Marker.h>
#include<pcl_ros/point_cloud.h>
#include<visualization_msgs/Marker.h>
#include"clustering/hz.h"
#include<std_msgs/ColorRGBA.h>
// polygon을 위한 header file
#include<geometry_msgs/Point32.h>
#include<geometry_msgs/Polygon.h>
#include<geometry_msgs/PolygonStamped.h>
#include<jsk_recognition_msgs/PolygonArray.h>

using namespace std;

class Get_data{
    public:
        Get_data() : seq_(0)
        {
            sub = nh.subscribe("/demansion_filterd_points", 1, &Get_data::get_data_callback, this);
            pub = nh.advertise<visualization_msgs::Marker>("Marker", 10);
            //pub2 = nh.advertise<clustering::hz>("/hz_configuration", 10);
            //pub3 = nh.advertise<jsk_recognition_msgs::PolygonArray>("/polygons",100);
            //pub_header_points = nh.advertise<visualization_msgs::MarkerArray>("Marker_header", 10);
        }
        void get_data_callback(const sensor_msgs::PointCloud2ConstPtr &ptr)
        {
            width = ptr->width;
            height = ptr->height;
            is_bigendian = ptr->is_bigendian;
            point_step = ptr->point_step;
            row_step = ptr->row_step;
            is_dense = ptr->is_dense;
            msg.fields = ptr->fields;
            vec_point.reserve(ptr->data.size());

            for(unsigned int k = 0; k< ptr->data.size();++k)
            {
                float X = 0.0;
                float Y = 0.0;
                //float Z = 0.0;

                unsigned int dataPosX = k*ptr->point_step + ptr->fields[0].offset; // x has an offset of 0
                //cout << ptr->point_step <<" "<< ptr->fields[0].offset <<" "<< dataPosX << " " << ptr->data.size() << endl;
                unsigned int dataPosY = k*ptr->point_step + ptr->fields[1].offset; // y has an offset of 4
                //cout << dataPosY << endl;
                //unsigned int dataPosZ = k*ptr->point_step + ptr->fields[2].offset; // z has an offset of 8
                //cout << dataPosZ << endl;
                
                memcpy(&X, &(ptr->data[dataPosX]), sizeof(float));
                memcpy(&Y, &(ptr->data[dataPosY]), sizeof(float));
                //memcpy(&Z, &(ptr->data[dataPosZ]), sizeof(float));
                
                //cout << "X : " << X << " Y : " << Y << " Z : " << Z << endl ;
                //cout << " K :  " << k << endl;

                if (X == 0 && Y == 0)
                    break;

                // 소수점 둘째짜리 까지 나타내기.
                //int X_sub = X * 100;
                //int Y_sub = Y * 100;
                //X = X_sub / 100.0;
                //Y = Y_sub / 100.0;
                
                // 반올림 수행
                X = round(X * 100) / 100.0;
                Y = round(Y * 100) / 100.0;
                
                //같은 점이 있나 확인.
                bool sm = find_same_point(X, Y);
                
                if(sm == false)
                {
                    pair<float, float> pt(X, Y);
                    vec_point.push_back(pt);
                }

            }
            
            //clustering 수행
            clustering();

            //cout << "--------------before filtering-----------" << endl;
            //print_cluster();
            //print_cluster_size();
            filtering_cluster();
            //filtering_point();
            //cout << "--------------after filtering-----------" << endl;
            print_cluster_size();
            //print_cluster();
            //cout << "받아온 point : " << vec_point.size() << "계산한 point : " << point_count << endl;
            for(int k = 0; k < 1000; ++k)
            {
                add_merging();
            }

            print_rviz_marker(); // 점으로 출력
            //find_8_point(); // polygon 출력

            hz_config.a = 1;
            pub2.publish(hz_config);
            //초기화
            vec_point.clear();
            header_vec.clear();
        }

        void clustering()
        {
            bool ok = false;
            int debuging_value = 0;

            vector<int> count_vec;
            count_vec.clear();

            for(int i = 0; i<vec_point.size(); ++i) // 받아오는 점들.
            {
                for(int j = 0; j< header_vec.size(); ++j) // 점점 채워지는 vector, header_vec의 맨앞은 각각의 subcluster의 중심점이다.
                {
                    if(header_vec.size() != 0)
                    {
                        ok = calc_Euclidean_distance(vec_point[i], header_vec[j][0]);
                        if(ok == true) // subcluster에 들어간다. // merge까지 수행해야댐.
                        {
                            count_vec.push_back(j);
                            ok = false;
                        }
                    }
                }

                if(count_vec.size() == 1) // 들어갈 수 있는 subcluster가 하나일 때
                {
                    //cout << "one cluster"<< endl;
                    header_vec[count_vec[0]].push_back(vec_point[i]);
                }

                else if( count_vec.size() > 1)// 들어갈 수 있는 subcluster가 여러 개 일때 merge 작업을 시작한다. // size가 더큰 subcluster로 합친다.
                {                    
                    //cout << "more than one cluster" << endl;
                    int best_cluster_idx = 0;
                    int temp = 0;

                    for(int k = 0; k< count_vec.size(); ++k)
                    {
                        // 가장 큰 subcluster 찾기
                        if(header_vec[count_vec[k]].size() > temp)
                        { 
                            best_cluster_idx = count_vec[k];
                            temp = header_vec[count_vec[k]].size();
                        }
                    }

                    //merge작업, 문제 없음.
                    for(int k = 0; k< count_vec.size(); ++k)
                    {
                        // best idx에 해당하는 subcluster는 내비둬야함.
                        if(count_vec[k] != best_cluster_idx)
                        {
                            merge_subcluster(count_vec[k], best_cluster_idx);
                        }
                    }
                    
                    //겹쳐지는 점 vector에 넣어주기 , 깜박해서 2시간동안 고생... 점 소실..
                    header_vec[best_cluster_idx].push_back(vec_point[i]);

                    //erase작업 // 주의 할 점. 뒤에서 부터 삭제해 가야한다.
                    for(int k = count_vec.size() - 1; k >=0; --k)
                    {                    
                        if(count_vec[k] != best_cluster_idx)
                            erase_subcluster(count_vec[k]);
                    }

                }
                else // 들어가지는 subcluster가 없을 때
                {
                    //cout << "none cluster" << endl;
                    make_subcluster(vec_point[i]); // 새로운 subcluster 생성.
                }
                count_vec.clear();

                // 50번째 점마다 갱신 , 10번째마다 하면 느려짐.
                if(i >= 50 && i % 50 == 0 )
                {
                    for(int k = 0; k<header_vec.size(); ++k)
                    {
                        move_header_point(k);
                    }
                }
                

            }
            //cout << "debuging_value : " << debuging_value << "  vec_point size : " << vec_point.size() << endl;
        }
        
        // point를 vector<pair>> 형태로 바꿔서 새로운 subcluster 생성.
        void make_subcluster(pair<float, float> input_point) 
        {
            vector< pair<float, float> > vec;
            vec.push_back(input_point);   
            header_vec.push_back(vec);
        }

        void merge_subcluster(int merge_header_idx, int best_cluster_idx)
        {
            // 하나의 subcluster로 병합
            header_vec[best_cluster_idx].insert(header_vec[best_cluster_idx].end(), header_vec[merge_header_idx].begin(), header_vec[merge_header_idx].end());
            
        }

        bool calc_Euclidean_distance(pair<float, float> input_point, pair<float, float> header_point)
        {
            float distance = 0.0;
            distance = sqrt( pow(input_point.first - header_point.first, 2) + pow(input_point.second - header_point.second, 2));
            if(distance < EUCLIDEAN_DISTANCE)
                return true;
            else
                return false;
        }

        // merge후 subcluster를 제거하는 함수.
        void erase_subcluster(int erase_header_idx)
        {
            header_vec.erase(header_vec.begin() + erase_header_idx);
        }

        //외곽선 추출을 위해 point 개수를 줄이는 함수. cluster의 내부 점들을 없애는 작업.
        void filtering_point()
        {
            for(int k = 0; k< header_vec.size(); ++k)
            {
                for(int i = 0; i<header_vec[k].size(); ++i)
                {
                    vector< pair<float, int> > vec_y;
                    pair<float, int> p;

                    p = make_pair(header_vec[k][i].first, i);
                    vec_y.push_back(p); // 처음 것을 넣어 둔다.

                    for(int m = i+1; m < header_vec[k].size(); ++m)
                    {
                        if(header_vec[k][i].first == header_vec[k][m].first)
                        {
                            p = make_pair(header_vec[k][m].first, m);
                            vec_y.push_back(p);
                        }
                    }
                    //같은 column에 있는 point 개수가 2개 이하일 경우 , 이 코드가 header_vec[k].size() 만큼 진행되야 한다.
                    if(vec_y.size() == 1 || vec_y.size() == 2)
                    {
                        vec_y.clear();
                    }
                        
                    else
                    {
                        // 한 column 정렬
                        sort(vec_y.begin(), vec_y.end(), compare);
                        // 맨 앞, 맨 뒤 제외한 모든 row 제거.
                        int no_erase_first_row = vec_y[0].second;
                        int no_erase_last_row = vec_y[vec_y.size()-1].second;

                        for(int a = vec_y.size() -1 ; a >= 0; --a)
                        {
                            if(vec_y[a].second == no_erase_first_row || vec_y[a].second == no_erase_last_row)
                                continue;
                            else    
                                header_vec[k].erase(header_vec[k].begin() + vec_y[a].second);
                        }
                        vec_y.clear();
                        i = 0; // 하나의 column의 filtering을 완료했으면 다시 처음부터 진행한다.
                    }
                }
            }
        }

        void filtering_cluster()
        {
            for(int k = header_vec.size() - 1; k>=0; --k)
            {
                if( header_vec[k].size() < cluster_threshold)
                    erase_subcluster(k);
            }
        }

        void move_header_point(int move_index)
        {
            float sum_x = 0;
            float sum_y = 0;
            float new_header_pointx = 0;
            float new_header_pointy = 0;

            for(int k = 0; k<header_vec[move_index].size(); ++k)
            {
                sum_x += header_vec[move_index][k].first;
                sum_y += header_vec[move_index][k].second;   
            }

            new_header_pointx = sum_x / header_vec[move_index].size();
            new_header_pointy = sum_y / header_vec[move_index].size();
            header_vec[move_index].insert(header_vec[move_index].begin(), make_pair(new_header_pointx, new_header_pointy));
        }

        static bool compare(pair<float, int> a, pair<float, int> b)
        {
            if(a.first == b.first)
                return a.second < b.second; 
            else 
                return a.first < b.first;
        }

        bool find_same_point(float X_, float Y_)
        {
            for(int k = 0; k<header_vec.size(); ++k)
            {
                for(int i = 0; i<header_vec[k].size(); ++i)
                {
                    if(header_vec[k][i].first == X_ && header_vec[k][i].second == Y_)
                    {
                        cout << "----find a same point----" << endl;
                        return true;
                    }
                }
            }
            return false;
        }

        // header_point를 중심으로 4등분을 하고 가장 멀리 떨어져있는 점을 찾는다. 각 분면당 2개의 점. 
        void find_8_point()
        {
            geometry_msgs::Point32 one_point32_x;
            geometry_msgs::Point32 one_point32_y;
            geometry_msgs::Point32 two_point32_x;
            geometry_msgs::Point32 two_point32_y;
            geometry_msgs::Point32 three_point32_x;
            geometry_msgs::Point32 three_point32_y;
            geometry_msgs::Point32 four_point32_x;
            geometry_msgs::Point32 four_point32_y;

            one_point32_x.x = 99999;
            one_point32_x.y = 99999;
            one_point32_y.x = 99999;
            one_point32_y.y = 99999;
            two_point32_x.x = 99999;
            two_point32_x.y = 99999;
            two_point32_y.x = 99999;
            two_point32_y.y = 99999;
            three_point32_x.x = 99999;
            three_point32_x.y = 99999;
            three_point32_y.x = 99999;
            three_point32_y.y = 99999;
            four_point32_x.x = 99999;
            four_point32_x.y = 99999;
            four_point32_y.x = 99999;
            four_point32_y.y = 99999;

            jsk_recognition_msgs::PolygonArray polygonArray_;
            std_msgs::Header header;

            

            unsigned int count = 0;

            for(int k = 0; k<header_vec.size(); ++k)
            {
                float one_big_x = 0.0;
                float one_big_y = 0.0;
                float two_big_x = 0.0;
                float two_big_y = 0.0;
                float three_big_x = 0.0;
                float three_big_y = 0.0;
                float four_big_x = 0.0;
                float four_big_y = 0.0;

                geometry_msgs::Polygon polygon_;
                geometry_msgs::PolygonStamped polygonstamped_;

                for(int i = 1; i < header_vec[k].size(); ++i) // 맨 처음 방( center 값 ) 을 제외, center를 중심으로 계산.
                {
                    // 1사분면
                    if(header_vec[k][i].first - header_vec[k][0].first >= 0 && header_vec[k][i].second - header_vec[k][0].second  >= 0)
                    {
                        if(one_big_x < fabs(header_vec[k][i].first - header_vec[k][0].first)) // 1사분면중 x가 가장 큰 좌표
                        {
                            one_big_x = fabs(header_vec[k][i].first);
                            one_point32_x.x = header_vec[k][i].first;
                            one_point32_x.y = header_vec[k][i].second;
                            one_point32_x.z = -1.785; 
                        }

                        else if(one_big_y < fabs(header_vec[k][i].second- header_vec[k][0].second)) // 1사분면중 y가 가장 큰 좌표, else if 로 함으로써 같은 좌표가 중복되게 들어가지 않도록함.
                        {
                            one_big_y = fabs(header_vec[k][i].second);
                            one_point32_y.x = header_vec[k][i].first;
                            one_point32_y.y = header_vec[k][i].second;
                            one_point32_y.z = -1.785;
                        }
                    }
                    //2사분면
                    else if(header_vec[k][i].first - header_vec[k][0].first <= 0 && header_vec[k][i].second - header_vec[k][0].second >= 0)
                    {
                        if(two_big_x < fabs(header_vec[k][i].first - header_vec[k][0].first)) // 2사분면중 x가 가장 큰 좌표
                        {
                            two_big_x = fabs(header_vec[k][i].first);
                            two_point32_x.x = header_vec[k][i].first;
                            two_point32_x.y = header_vec[k][i].second;
                            two_point32_x.z = -1.785; 
                        }

                        else if(two_big_y < fabs(header_vec[k][i].second - header_vec[k][0].second)) // 2사분면중 y가 가장 큰 좌표, else if 로 함으로써 같은 좌표가 중복되게 들어가지 않도록함.
                        {
                            two_big_y = fabs(header_vec[k][i].second);
                            two_point32_y.x = header_vec[k][i].first;
                            two_point32_y.y = header_vec[k][i].second;
                            two_point32_y.z = -1.785;
                        }
                    }
                    //3사분면
                    else if(header_vec[k][i].first - header_vec[k][0].first <= 0 && header_vec[k][i].second - header_vec[k][0].second <= 0)
                    {
                        if(three_big_x < fabs(header_vec[k][i].first - header_vec[k][0].first )) // 3사분면중 x가 가장 큰 좌표
                        {
                            three_big_x = fabs(header_vec[k][i].first);
                            three_point32_x.x = header_vec[k][i].first;
                            three_point32_x.y = header_vec[k][i].second;
                            three_point32_x.z = -1.785; 
                        }

                        else if(three_big_y < fabs(header_vec[k][i].second - header_vec[k][0].second )) // 3사분면중 y가 가장 큰 좌표, else if 로 함으로써 같은 좌표가 중복되게 들어가지 않도록함.
                        {
                            three_big_y = fabs(header_vec[k][i].second);
                            three_point32_y.x = header_vec[k][i].first;
                            three_point32_y.y = header_vec[k][i].second;
                            three_point32_y.z = -1.785;
                        }
                    }
                    //4사분면
                    else if(header_vec[k][i].first - header_vec[k][0].first >= 0 && header_vec[k][i].second - header_vec[k][0].second <= 0)
                    {
                        if(four_big_x < fabs(header_vec[k][i].first - header_vec[k][0].first )) // 4사분면중 x가 가장 큰 좌표
                        {
                            four_big_x = fabs(header_vec[k][i].first);
                            four_point32_x.x = header_vec[k][i].first;
                            four_point32_x.y = header_vec[k][i].second;
                            four_point32_x.z = -1.785; 
                        }

                        else if(four_big_y < fabs(header_vec[k][i].second - header_vec[k][0].second )) // 4사분면중 y가 가장 큰 좌표, else if 로 함으로써 같은 좌표가 중복되게 들어가지 않도록함.
                        {
                            four_big_y = fabs(header_vec[k][i].second);
                            four_point32_y.x = header_vec[k][i].first;
                            four_point32_y.y = header_vec[k][i].second;
                            four_point32_y.z = -1.785;
                        }
                    }
                }//한 subcluster 종료
                //polygon 만들기
                polygonArray_.header.seq = seq_++;
                polygonArray_.header.stamp = ros::Time::now();
                polygonArray_.header.frame_id = "velodyne";

                //direction important!!! 순서 주의
                if(one_point32_x.x != 99999 && one_point32_x.y != 99999)
                    polygon_.points.push_back(one_point32_x);
                else
                    cout << "----one_point32_x error----" << endl;
                if(one_point32_y.x != 99999 && one_point32_y.y != 99999)    
                    polygon_.points.push_back(one_point32_y);
                else
                    cout << "----one_point32_y error----" << endl;
                if(two_point32_y.x != 99999 && two_point32_y.y != 99999)
                    polygon_.points.push_back(two_point32_y);
                else
                    cout << "----two_point32_y error----" << endl;
                if(two_point32_x.x != 99999 && two_point32_x.y != 99999)    
                    polygon_.points.push_back(two_point32_x);                    
                else
                    cout << "----two_point32_x error----" << endl;
                if(three_point32_x.x != 99999 && three_point32_x.y != 99999)
                    polygon_.points.push_back(three_point32_x);
                else
                    cout << "----three_point32_x error----" << endl;
                if(three_point32_y.x != 99999 && three_point32_y.y != 99999)
                    polygon_.points.push_back(three_point32_y);
                else
                    cout << "----three_point32_y error----" << endl;
                if(four_point32_y.x != 99999 && four_point32_y.y != 99999)
                    polygon_.points.push_back(four_point32_y); 
                else
                    cout << "----four_point32_y error----" << endl;
                if(four_point32_x.x != 99999 && four_point32_x.y != 99999)
                    polygon_.points.push_back(four_point32_x);
                else
                    cout << "----four_point32_x error----" << endl;
                   
                

                polygonstamped_.header.seq = polygonArray_.header.seq;
                polygonstamped_.header.stamp = polygonArray_.header.stamp;
                polygonstamped_.header.frame_id = polygonArray_.header.frame_id;
                polygonstamped_.polygon = polygon_;

                polygonArray_.polygons.push_back(polygonstamped_);
                polygonArray_.labels.push_back(count++);
                polygonArray_.likelihood.push_back(1.0);

                polygon_.points.clear();
            }

            pub3.publish(polygonArray_);

            polygonArray_.polygons.clear();
            polygonArray_.labels.clear();
            polygonArray_.likelihood.clear();
        }

        void add_merging()
        {
            float X_center = 0.0;
            float Y_center = 0.0;
            
            

            for(int k = 0; k< header_vec.size(); ++k)
            {
                for(int i = k+1; i < header_vec.size(); ++i)
                {
                    bool check_1 = false;
                    bool check_2 = false;
                    
                    pair<float, float> p;

                    X_center = (header_vec[k][0].first + header_vec[i][0].first) / 2;
                    Y_center = (header_vec[k][0].second + header_vec[i][0].second) / 2;
                    p = make_pair(X_center, Y_center);
                    check_1 = calc_Euclidean_distance(p, header_vec[k][0]);
                    check_2 = calc_Euclidean_distance(p, header_vec[i][0]);

                    if(check_1 == true && check_2 == true) // 두개가 겹친다면
                    {
                        merge_subcluster(i, k); // i를 k에 합병.
                        erase_subcluster(i);
                        for(int m = 0; m < header_vec.size(); ++m)
                            move_header_point(m);
                        i--; // 다시 그자리부터 시작.
                    } 
                }
            }
        }
        void print_cluster()
        {
            point_count = 0;

            for(int k = 0; k<header_vec.size(); ++k)
            {
                //cout << k << "번째 cluster size = " << header_vec[k].size() << endl;
                for(int i = 0; i<header_vec[k].size(); ++i)
                {
                    if(k == 0)
                        cout << "x = " << header_vec[k][i].first << " y = " << header_vec[k][i].second << endl; 
                    point_count++;
                }
            }
        }
        void print_cluster_size()
        {
            point_count = 0;

            for(int k = 0; k< header_vec.size(); ++k)
            {
                cout << k << "번째 cluster size : " << header_vec[k].size() << endl;
                for(int i = 0; i< header_vec[k].size(); ++i)
                {
                    point_count++;
                }
            }
        }

        void print_rviz_marker()
        {
            visualization_msgs::Marker points;
            // visualization_msgs::MarkerArray header_sphere;
            // visualization_msgs::Marker header_points;

            std_msgs::ColorRGBA c;

            int id = 0;
            int header_id = 0;
            int count = 0;
            c.a = 1.0; 
            for(int k = 0; k<header_vec.size(); ++k)
            {
                for(int i = 0; i < header_vec[k].size(); ++i)
                {
                    points.header.frame_id = "velodyne";
                    points.header.stamp = ros::Time::now();
                    points.ns = "points";
                    points.action = visualization_msgs::Marker::ADD;
                    points.pose.orientation.w = 1.0;
                    points.lifetime = ros::Duration(0.1);
                    points.id = id++;
                    points.type = visualization_msgs::Marker::POINTS;
                    points.scale.x = 0.1;
                    points.scale.y = 0.1;

                    points.color.r = 0.0;
                    points.color.g = 1.0;
                    points.color.b = 0.0;
                    points.color.a = 1.0;

                    geometry_msgs::Point p;


                    p.x = header_vec[k][i].first;
                    p.y = header_vec[k][i].second;
                    p.z = -1.785;

                    // vector 별 color 넣기
                    switch(count % 15)
                    {

                        case 0 : 
                            c.r = 0.5;
                            c.g = 0.5;
                            c.b = 0.5;
                            break;
                        case 1 : 
                            c.r = 0.5;
                            c.g = 0.0;
                            c.b = 0.0;
                            break;
                        case 2 : 
                            c.r = 0.0;
                            c.g = 0.5;
                            c.b = 0.0;
                            break;
                        case 3 : 
                            c.r = 0.0;
                            c.g = 0.0;
                            c.b = 0.5;
                            break;
                        case 4 : 
                            c.r = 0.5;
                            c.g = 0.5;
                            c.b = 0.0;
                            break;
                        case 5 : 
                            c.r = 0.5;
                            c.g = 0.0;
                            c.b = 0.5;
                            break;
                        case 6 :
                            c.r = 0.0;
                            c.g = 0.5;
                            c.b = 0.5; 
                            break;
                        case 7 :
                            c.r = 1.0;
                            c.g = 1.0;
                            c.b = 1.0; 
                            break;
                        case 8 :
                            c.r = 1.0;
                            c.g = 0.0;
                            c.b = 0.0;  
                            break;
                        case 9 :
                            c.r = 0.0;
                            c.g = 1.0;
                            c.b = 0.0;  
                            break;
                        case 10 :
                            c.r = 0.0;
                            c.g = 0.0;
                            c.b = 1.0;  
                            break;
                        case 11 :
                            c.r = 1.0;
                            c.g = 1.0;
                            c.b = 0.0;  
                            break;
                        case 12 :
                            c.r = 1.0;
                            c.g = 0.0;
                            c.b = 1.0;  
                            break;
                        case 13 :
                            c.r = 0.0;
                            c.g = 1.0;
                            c.b = 1.0;  
                            break; 
                        case 14 :
                            c.r = 0.0;
                            c.g = 0.0;
                            c.b = 0.0;  
                            break;                                                                                                                                                                                                                                                                                                                                                                   
                    }

                    points.points.push_back(p);
                    points.colors.push_back(c);
                }
                count++;
                if(count > 15)
                    count = 0;
            }
            pub.publish(points);
            points.points.clear();
            points.colors.clear();
            /*for(int k = 0; k<header_vec.size(); ++k)
            {
                header_points.header.frame_id = "velodyne";
                header_points.header.stamp = ros::Time::now();
                header_points.ns = "header_points";
                header_points.action = visualization_msgs::Marker::ADD;
                header_points.pose.orientation.w = 1.0;
                header_points.lifetime = ros::Duration(0.1);
                header_points.id = header_id++;
                header_points.type = visualization_msgs::Marker::SPHERE;
                header_points.pose.position.x = header_vec[k][0].first;
                header_points.pose.position.y = header_vec[k][0].second;
                header_points.pose.position.z = -1.785;

                header_points.scale.x = 1.0;
                header_points.scale.y = 1.0;
                header_points.scale.z = 1.0;

                header_points.color.r = 0.0f;
                header_points.color.g = 1.0f;
                header_points.color.b = 1.0f;
                header_points.color.a = 1.0;      
                header_sphere.markers.push_back(header_points);                                     
            }
            pub_header_points.publish(header_points);*/
        }
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub2;
        ros::Publisher pub3;
        ros::Publisher pub_header_points;

        // point 정보를 받아오기 위한 vector , callback 함수에서 사용.
        vector< pair<float, float> > vec_point;

        //subcluster들을 넣어두기 위한 header vector 
        //각각의 subcluster의 맨 앞에 있는 x, y 가 subcluster의 중심점이 된다.
        vector< vector< pair<float, float> > > header_vec;
        sensor_msgs::PointCloud2 msg;
        clustering::hz hz_config;
        int seq_;
        bool same;
        int width;
        int height;
        bool is_bigendian;
        int point_step;
        int row_step;
        bool is_dense;
        int point_count;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "clustering");
    Get_data gd;
    ros::spin();
}

