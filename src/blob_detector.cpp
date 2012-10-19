#include <iostream>
#include <thread>
#include <cmath>
#include <queue>
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Vector3.h"

#define M_TAU (2 * M_PI)

using namespace std;

class blob {
public:
    blob();

    void add_pixel(const uint32_t &x, const uint32_t &y);

    uint32_t n;
    double x_m2, y_m2;
    double x_mean, y_mean;
    double x_var, y_var;
};

blob::blob():
    n(0),
    x_m2(0),    y_m2(0),
    x_mean(0),  y_mean(0),
    x_var(0),   y_var(0) {
};
    
void blob::add_pixel(const uint32_t &x, const uint32_t &y) {
    double delta;
    n++;

    delta = x - x_mean;
    x_mean += delta / n;
    x_m2 += delta * (x - x_mean);
    x_var = x_m2 / n;

    delta = y - y_mean;
    y_mean += delta / n;
    y_m2 += delta * (y - y_mean);
    y_var = y_m2 / n;
};

ros::NodeHandle *node;
ros::Subscriber *sub_image_source;
ros::Subscriber *sub_color_source;
ros::Publisher *pub_debug;
ros::Publisher *pub_locations;

string nodename;

void receive_image_source(const sensor_msgs::Image::ConstPtr &msg) {
    static int *blob_index = NULL;
    static uint32_t w = 0;
    static uint32_t h = 0;

    if(! blob_index || msg->width != w || msg->height != h) {
        w = msg->width;
        h = msg->height;
        delete blob_index;

        blob_index = new int[w * h];
    };

    double hue_threshold;
    double sat_threshold;
    double target_hue;
    node->getParam(nodename + "/hue_threshold", hue_threshold);
    node->getParam(nodename + "/sat_threshold", sat_threshold);
    node->getParam(nodename + "/target_hue",    target_hue);

    cout << "target_hue: " << target_hue << endl;

    sensor_msgs::Image out;
    out.header.stamp = msg->header.stamp;
    out.header.seq = msg->header.seq;
    out.width = w;
    out.height = h;
    out.encoding = msg->encoding;
    out.step = msg->step;
    out.data = vector<uint8_t>(msg->step * h);
    vector<blob> blobs;
    
    int r_off;
    int g_off;
    int b_off;
    if(out.encoding == "rgb8") {
        r_off = 0;
        g_off = 1;
        b_off = 2;
    }
    else if(out.encoding == "bgr8") {
        r_off = 2;
        g_off = 1;
        b_off = 0;
    }
    else {
        cout << "Unknown encoding: " << out.encoding << endl;
        return;
    };

    for(uint32_t x = 0; x < msg->width; x++) {
        for(uint32_t y = 0; y < msg->height; y++) {
            uint32_t r = msg->data[3 * x + y * msg->step + r_off];
            uint32_t g = msg->data[3 * x + y * msg->step + g_off];
            uint32_t b = msg->data[3 * x + y * msg->step + b_off];
            double dr = (double)r / 256.0;
            double dg = (double)g / 256.0;
            double db = (double)b / 256.0;
            double alph = dr - (dg + db) / 2;
            double beta = (dg - db) * sqrt(3) / 2;
            double hue = atan2(beta, alph) / M_TAU;
            double sat = sqrt(alph * alph + beta * beta);
            bool hue_good = abs(fmod(hue - target_hue + 0.5, 1) - 0.5) < hue_threshold / 2;
            bool sat_good = sat > sat_threshold;

            //blob_index[x + y * w] = -1;
            
            if(hue_good && sat_good)
                blob_index[x + y * w] = -2;
            else
                blob_index[x + y * w] = -1;
                
/*
            if(hue_good && sat_good) {
                if(x > 0 && blob_index[x - 1 + y * w] != -1)
                    blob_index[x + y * w] = blob_index[x - 1 + y * w];
                else if(y > 0 && blob_index[x + (y - 1) * w] != -1)
                    blob_index[x + y * w] = blob_index[x + (y - 1) * w];
                else {
                    blob_index[x + y * w] = blobs.size();
                    blobs.push_back(blob());
                };
                blobs[blob_index[x + y * w]].add_pixel(x, y);
            };
*/

            out.data[3 * x + y * out.step + r_off] = (uint32_t)r / 2 + (hue_good ? 128 : 0);
            out.data[3 * x + y * out.step + g_off] = (uint32_t)g / 2;
            out.data[3 * x + y * out.step + b_off] = (uint32_t)b / 2 + (sat_good ? 128 : 0);
            //out.data[3 * x + y * out.step + 0] = r;
            //out.data[3 * x + y * out.step + 1] = g;
            //out.data[3 * x + y * out.step + 2] = b;
        };
    };

    queue<uint32_t> qx;
    queue<uint32_t> qy;
    for(uint32_t x = 0; x < msg->width; x++) {
        for(uint32_t y = 0; y < msg->height; y++) {
            if(blob_index[x + y * w] == -2) {
                uint32_t index = blobs.size();
                blobs.push_back(blob());

                qx.push(x);
                qy.push(y);
                for(; qx.size();) {
                    uint32_t tx = qx.front();
                    uint32_t ty = qy.front();
                    qx.pop();
                    qy.pop();
                    if(blob_index[tx + ty * w] == -2) {
                        blob_index[tx + ty * w] = index;
                        blobs[index].add_pixel(tx, ty);
                        if(tx < msg->width - 1) {
                            qx.push(tx + 1);
                            qy.push(ty);
                        };
                        if(tx < msg->height - 1) {
                            qx.push(tx);
                            qy.push(ty + 1);
                        };
                        if(tx > 0) {
                            qx.push(tx - 1);
                            qy.push(ty);
                        };
                        if(ty > 0) {
                            qx.push(tx);
                            qy.push(ty - 1);
                        };
                    };
                };
            };
        };
    };
    pub_debug->publish(out);

    int size_threshold;
    node->getParam(nodename + "/size_threshold", size_threshold);
    //cout << "Publishing " << blobs.size() << " blobs." << endl;
    for(uint32_t i = 0; i < blobs.size(); i++) {
        if(blobs[i].n > (uint32_t)size_threshold) {
            geometry_msgs::Vector3 p;
            p.x = blobs[i].x_mean - w / 2;
            p.y = blobs[i].y_mean - h / 2;
            p.z = 1 / sqrt(blobs[i].x_var + blobs[i].y_var);
            pub_locations->publish(p);
        };
    };
};

void receive_color_source(const std_msgs::ColorRGBA::ConstPtr &msg) {
    double alph = msg->r - (msg->g + msg->b) / 2;
    double beta = (msg->g - msg->b) * sqrt(3) / 2;
    double hue = atan2(beta, alph) / M_TAU;
    node->setParam(nodename + "/target_hue", hue);
    cout << "Received color: " << msg->r << ", " << msg->g << ", " << msg->b << "; hue: " << hue << endl;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "blob_detector", ros::init_options::AnonymousName);

    node = new ros::NodeHandle();
    nodename = ros::this_node::getName();
    
    pub_debug = new ros::Publisher(node->advertise<sensor_msgs::Image>(nodename + "/debug_video", 1));
    pub_locations = new ros::Publisher(node->advertise<geometry_msgs::Vector3>(nodename + "/blob_locations", 128));

    if(! node->hasParam(nodename + "/hue_threshold"))
         node->setParam(nodename + "/hue_threshold", 0.04);
    if(! node->hasParam(nodename + "/sat_threshold"))
         node->setParam(nodename + "/sat_threshold", 0.8);
    if(! node->hasParam(nodename + "/target_hue"))
         node->setParam(nodename + "/target_hue", 0);
    if(! node->hasParam(nodename + "/size_threshold"))
         node->setParam(nodename + "/size_threshold", 1024);
    if(! node->hasParam(nodename + "/image_source"))
         node->setParam(nodename + "/image_source", "image");
    if(! node->hasParam(nodename + "/color_source"))
         node->setParam(nodename + "/color_source", "color");

    node->setParam(nodename + "/hue_threshold__meta/type", "float");
    node->setParam(nodename + "/hue_threshold__meta/min_value", 0.0);
    node->setParam(nodename + "/hue_threshold__meta/max_value", 1.0);
    node->setParam(nodename + "/hue_threshold__meta/increment", 0.001);

    node->setParam(nodename + "/sat_threshold__meta/type", "float");
    node->setParam(nodename + "/sat_threshold__meta/min_value", 0.0);
    node->setParam(nodename + "/sat_threshold__meta/max_value", 1.0);
    node->setParam(nodename + "/sat_threshold__meta/increment", 0.001);

    node->setParam(nodename + "/target_hue__meta/type", "float");
    node->setParam(nodename + "/target_hue__meta/min_value", -0.5);
    node->setParam(nodename + "/target_hue__meta/max_value", 0.5);
    node->setParam(nodename + "/target_hue__meta/increment", 0.001);

    node->setParam(nodename + "/size_threshold__meta/type", "float");
    node->setParam(nodename + "/size_threshold__meta/min_value", 0);
    node->setParam(nodename + "/size_threshold__meta/max_value", 500000);
    
    node->setParam(nodename + "/image_source__meta/type", "string");
    node->setParam(nodename + "/image_source__meta/defines", "topic");
    node->setParam(nodename + "/image_source__meta/topic_type", "sensor_msgs/Image");

    node->setParam(nodename + "/color_source__meta/type", "string");
    node->setParam(nodename + "/color_source__meta/defines", "topic");
    node->setParam(nodename + "/color_source__meta/topic_type", "std_msgs/ColorRGBA");
    
    string image_source;
    string color_source;
    ros::Rate r(100);
    while(ros::ok()) {
        string new_image_source;
        string new_color_source;

        if(node->getParam(nodename + "/image_source", new_image_source)) {
            if(new_image_source != image_source) {
                image_source = new_image_source;
                if(sub_image_source) {
                    delete sub_image_source;
                    sub_image_source = NULL;
                };
                sub_image_source = new ros::Subscriber(node->subscribe(image_source, 1, receive_image_source));
            };
        };
        if(node->getParam(nodename + "/color_source", new_color_source)) {
            if(new_color_source != color_source) {
                color_source = new_color_source;
                if(sub_color_source) {
                    delete sub_color_source;
                    sub_color_source = NULL;
                };
                sub_color_source = new ros::Subscriber(node->subscribe(color_source, 1, receive_color_source));
            };
        };

        ros::spinOnce();
        //this_thread::yield();
        r.sleep();
    };

    node->deleteParam(nodename);

    return 0;
};

