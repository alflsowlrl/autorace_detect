void resample(char * arr, float * weight){
    //for new particles
    int len = 4;
    char new_arr[len];

    //uniform random number 
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    float r = distribution(generator)/len;
    
    float c = weight[0];
    float U = 0;
    int i = 0;

    for(int j = 0; j < len; j++){
        U = r + float(j)/len;
        while(U > c){
            i++;
            c += weight[i];
        }
        ROS_INFO("U: %f c: %f i: %d", U, c, i);
        new_arr[j] = arr[i];
    }

    for(int i = 0; i < len; i++)
        ROS_INFO("particle: %c", new_arr[i]);

}

int main(int argc, char ** argv){
    ros::init(argc, argv, "opencv_slam");
    ros::Duration nap(0.1);
    ros::NodeHandle nh;
    // Slam slam(nh, 30);


    // while(ros::ok()){  
    //     ros::spinOnce();
    //     nap.sleep();
    // }

    char arr[] = {'a', 'b', 'c', 'd'};
    float weight[] = {0.4, 0.2, 0.2, 0.2};
    resample(arr, weight);
    return 0;
}