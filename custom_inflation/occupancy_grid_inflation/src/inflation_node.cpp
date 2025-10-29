#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_interfaces/srv/inflation_grid.hpp"     // CHANGE

class OccupancyGridInflation : public rclcpp::Node {
public:

    OccupancyGridInflation() : Node("occupancy_grid_inflation") {
        std::cout << "HELLO" << std::endl;

        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&OccupancyGridInflation::mapCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_occ", 10);

        srv_ = this->create_service<map_interfaces::srv::InflationGrid>(
            "inflation_grid_service", 
            std::bind(&OccupancyGridInflation::handleInflationRequest, this, std::placeholders::_1, std::placeholders::_2));
        
        latest_grid_ = nullptr;
         // if you change this name make sure you also change the goal_selection_node  inflation_grid_service
    }


private:
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;  // Store the latest received grid
    // void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    //     auto inflated_grid = *msg;
    //     // inflateObstacles(inflated_grid, 17, 0.75);
    //     pub_->publish(inflated_grid);
    //     std::cout << "got map update"    << std::endl;
    //     latest_grid_ = msg;

    // }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        nav_msgs::msg::OccupancyGrid inflated_grid = *msg;
        pub_->publish(inflated_grid);

        inflateObstacles(inflated_grid, 10, 20, 0.9); // 10
        inflated_grid.header.frame_id = "odom";
        // inflated_grid.header.stamp = this->now();
        inflated_grid.info.origin.position.x = -14 * inflated_grid.info.resolution;
        inflated_grid.info.origin.position.y = -76 * inflated_grid.info.resolution;
        // inflated_grid.info.origin.position.z = 0;
        // inflated_grid.info.origin.orientation.x = 0;
        // inflated_grid.info.origin.orientation.y = 0;
        // inflated_grid.info.origin.orientation.z = 0.7071068;  // sin(90°/2)
        // inflated_grid.info.origin.orientation.w = 0.7071068;  // cos(90°/2)
    
        RCLCPP_INFO(this->get_logger(), "Grid resolution: %f", inflated_grid.info.resolution);
        RCLCPP_INFO(this->get_logger(), "Grid width: %d, height: %d", inflated_grid.info.width, inflated_grid.info.height);
        std::cout << "got map update"    << std::endl;
        latest_grid_ = msg;


    }



    void handleInflationRequest(
        const std::shared_ptr<map_interfaces::srv::InflationGrid::Request> request,
        std::shared_ptr<map_interfaces::srv::InflationGrid::Response> response) 
    {   
        std::cout << "Received service request" << std::endl;
        if (!latest_grid_) {
            RCLCPP_WARN(this->get_logger(), "No occupancy grid received yet!");
            response->occupancy_grid = nav_msgs::msg::OccupancyGrid();
            response->robot_pose_x = -1;
            response->robot_pose_y = -1;
            return;
        }

        auto inflated_grid = *latest_grid_;  
        inflateObstacles(inflated_grid, 10, 20 , 0.90);  // Inflate obstacles
        // pub_->publish(inflated_grid);  // Publish inflated map

        response->occupancy_grid = inflated_grid;
        response->robot_pose_x = 14;  // Placeholder values
        response->robot_pose_y = 76;
        // response_promise.set_value(response);
        std::cout << "Received service given" << std::endl;
        return;
    }


  

    

    void inflateObstacles(nav_msgs::msg::OccupancyGrid &grid, int radius, int add_on, double decrease_factor) {
        int width = grid.info.width;
        int height = grid.info.height;
        // std::cout << "width: " << width << " height: " << height << std::endl;
        std::vector<int8_t> new_data = grid.data;
        // for (int x = 0; x < width; x++) {
        //     int sum = 0;
        //     for (int y = 0; y < height ; y++) {
        //         sum += int(grid.data[y * width + x]);
        //     }
        //     std::cout << "col: " << x << "sum: " << sum << std::endl;

        // }
            
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (grid.data[y * width + x] > 50) { 
                    for (int dy = -(radius+add_on); dy <= radius+add_on; dy++) {
                        for (int dx = -(radius+add_on); dx <= radius+add_on; dx++) {
                            

                                                        
                            int real_radius = std::sqrt(std::abs(dx) + std::abs(dy));
                            int new_value = int(grid.data[y * width + x])  * (std::pow(decrease_factor,real_radius)); 
                            int nx = x + dx;
                            int ny = y + dy;

                            
                            // 26 is the end of CV deadzone
                            // 140 is an estimate of the top deadzone but that doesn't matter
                            bool canNotGoInNeg = false;
                            if( nx >= 26 ){
                                canNotGoInNeg = true;
                            }

                            

                            // bool negOneCheck =  false;

                            // if (!canGoInNeg && grid.data[ny * width + nx] != -1) {
                            //     negOneCheck = true;
                            // }

                            
                            if (nx >= 0 && ny >= 0 && nx < width && ny < height && canNotGoInNeg) {
                                if(ny >= 140){
                                    new_data[ny * width + nx] = 100;
                                }

                                                                            
                                else if( grid.data[y * width + x] == 100  && std::abs(dx) < radius && std::abs(dy)  < radius){
                                    new_data[ny * width + nx] = 100;
                                }
                                else{
                                    
                                    new_data[ny * width + nx] = std::max(new_data[ny * width + nx], int8_t(new_value));
                                    // new_data[ny * width + nx] = new_data[ny * width + nx];  

                                }
                            }


                        }
                    }
                }
            }
        }
        grid.data = new_data;
    }

    // void inflateObstacles(nav_msgs::msg::OccupancyGrid &grid, int radius, int add_on ,double decrease_factor) {

    //     std::cout << "add on: " << add_on << std::endl;
    //     int width = grid.info.width;
    //     int height = grid.info.height;

    //     std::vector<int8_t> new_data = grid.data;

    //     for (int y = 0; y < height; y++) {
    //         for (int x = 0; x < width; x++) {
    //             if (grid.data[y * width + x] > 50) { 
    //                 for (int dy = -radius; dy <= radius; dy++) {
    //                     for (int dx = -radius; dx <= radius; dx++) {
                            
    //                         int real_radius = std::sqrt(std::abs(dx) + std::abs(dy));
    //                         int new_value = int(grid.data[y * width + x])  * (std::pow(decrease_factor,real_radius)); 
    //                         int nx = x + dx;
    //                         int ny = y + dy;
    //                         if (nx >= 0 && ny >= 0 && nx < width && ny < height && grid.data[ny * width + nx] != -1) {
    //                             new_data[ny * width + nx] = std::max(new_data[ny * width + nx], int8_t(new_value));
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     grid.data = new_data;
    // }


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::Service<map_interfaces::srv::InflationGrid>::SharedPtr srv_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridInflation>());
    rclcpp::shutdown();
    return 0;
}
    
