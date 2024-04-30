    #include <iostream>
    #include <Eigen>
    using namespace std;
    using namespace Eigen;
   
    int main() {

        Matrix3d C_W_R; // The rotation matrix from the camera to the workpiece
        C_W_R << 0.93, -0.353, 0.0669,
            0.3535, 0.8660, -0.35355,
            0.669783, 0.35355, 0.933;

        Matrix3d W_C = C_W_R.inverse();  // The rotation matrix from the workpiece to the camera
 

        Matrix3d B_C; // The rotation matrix from the base to the camera
        B_C << -1, 0, 0,
            0, 0, -1,
            0, -1, 0;

        Matrix3d C_B = B_C.inverse(); // The rotation matrix from the camera to the base

        // The position of each hole relative to the fiducial
        Vector3d Wp_h1(0.2, 0.2, 0);
        Vector3d Wp_h2(0.2, -0.2, 0);
        Vector3d Wp_h3(-0.2, 0.2, 0);
        Vector3d Wp_h4(-0.2, -0.2, 0);
    
        // The position of workpiece relative to the camera
        Vector3d PW(0.2, 0.0, 0.5); 
    
        // The position of the camera relative to the base
        Vector3d PC(0.0, -0.7, 0.7);

        // Transform hole position to robot base frame
        Vector3d Bp_h1 = C_B * (W_C * Wp_h1 + PW) + PC;
        Vector3d Bp_h2 = C_B * (W_C * Wp_h2 + PW) + PC;
        Vector3d Bp_h3 = C_B * (W_C * Wp_h3 + PW) + PC;
        Vector3d Bp_h4 = C_B * (W_C * Wp_h4 + PW) + PC;

        cout << "///////////////////////  the pose of the holes relative to robot base frame  //////////////////////////////////\n" << endl;


        cout << "The position of hole 1 relative to the base: " << Bp_h1.transpose() << endl;
        cout << "The position of hole 2 relative to the base: " << Bp_h2.transpose() << endl;
        cout << "The position of hole 3 relative to the base: " << Bp_h3.transpose() << endl;
        cout << "The position of hole 4 relative to the base: " << Bp_h4.transpose() << endl;

        cout << endl;
        cout << endl;
        cout << endl;


        ///////////////////////   Target pose part  //////////////////////////////////

    
        Vector3d camera_pose(0, 0, 0.1);


       // Transform Hole Position to the Camera Frame
       // Calculate the positions of 4 holes in the camera frame

        Vector3d Cp_h1 = W_C * Wp_h1 + PW;
        Vector3d Cp_h2 = W_C * Wp_h2 + PW;
        Vector3d Cp_h3 = W_C * Wp_h3 + PW;
        Vector3d Cp_h4 = W_C * Wp_h4 + PW;

        

        // Calculate New  Camera Position in Camera Frame
        Vector3d N_CpC_h1 = Cp_h1 - camera_pose;
        Vector3d N_CpC_h2 = Cp_h2 - camera_pose;
        Vector3d N_CpC_h3 = Cp_h3 - camera_pose;
        Vector3d N_CpC_h4 = Cp_h4 - camera_pose;

       
       // Transform Desired Camera Position to the Robot Base Frame

        Vector3d N_BpC_h1 = C_B * N_CpC_h1 + PC;
        Vector3d N_BpC_h2 = C_B * N_CpC_h2 + PC;
        Vector3d N_BpC_h3 = C_B * N_CpC_h3 + PC;
        Vector3d N_BpC_h4 = C_B * N_CpC_h4 + PC;

        cout << "///////////////////////  The Camera Target Pose Part  //////////////////////////////////\n" << endl;

        cout << "The position of camera target pose of hole 1 relative to the base: " << N_BpC_h1.transpose() << endl;
        cout << "The position of camera target pose of hole 2 relative to the base: " << N_BpC_h2.transpose() << endl;
        cout << "The position of camera target pose of hole 3 relative to the base: " << N_BpC_h3.transpose() << endl;
        cout << "The position of camera target pose of hole 4 relative to the base: " << N_BpC_h4.transpose()<<"\n" << endl;

        // Compute Desired Camera Orientation


        Vector3d Direction_Vector_h1 = N_BpC_h1 - PC;

        Vector3d Direction_Vector_h2 = N_BpC_h2 - PC;
        Vector3d Direction_Vector_h3 = N_BpC_h3 - PC;
        Vector3d Direction_Vector_h4 = N_BpC_h4 - PC;


        // Linearization

        Vector3d Normalized_Direction_Vector_h1 = Direction_Vector_h1.normalized();
        Vector3d Normalized_Direction_Vector_h2 = Direction_Vector_h2.normalized();
        Vector3d Normalized_Direction_Vector_h3 = Direction_Vector_h3.normalized();
        Vector3d Normalized_Direction_Vector_h4 = Direction_Vector_h4.normalized();


       

        Vector3d Perpendicular_Vector(1.00000, 0, 0);

        Vector3d Cross_Product_Vector_h1 = Normalized_Direction_Vector_h1.cross(Perpendicular_Vector);
        Vector3d Cross_Product_Vector_h2 = Normalized_Direction_Vector_h2.cross(Perpendicular_Vector);
        Vector3d Cross_Product_Vector_h3 = Normalized_Direction_Vector_h3.cross(Perpendicular_Vector);
        Vector3d Cross_Product_Vector_h4 = Normalized_Direction_Vector_h4.cross(Perpendicular_Vector);

      

        // constructing the rotation matrices of target pose for the camera

        MatrixXd target_pose_h1(3, 3);
        target_pose_h1 << Cross_Product_Vector_h1, Perpendicular_Vector, Normalized_Direction_Vector_h1;

        MatrixXd target_pose_h2(3, 3);
        target_pose_h2 << Cross_Product_Vector_h2, Perpendicular_Vector, Normalized_Direction_Vector_h2;

        MatrixXd target_pose_h3(3, 3);
        target_pose_h3 << Cross_Product_Vector_h3, Perpendicular_Vector, Normalized_Direction_Vector_h3;

        MatrixXd target_pose_h4(3, 3);
        target_pose_h4 << Cross_Product_Vector_h4, Perpendicular_Vector, Normalized_Direction_Vector_h4;

        

        cout << "The rotation matrix of camera target pose of hole 1 relative to the base: \n" << target_pose_h1 << "\n" << endl;
        cout << "The rotation matrix of camera target pose of hole 2 relative to the base: \n" << target_pose_h2 << "\n" << endl;
        cout << "The rotation matrix of camera target pose of hole 3 relative to the base: \n" << target_pose_h3 << "\n" << endl;
        cout << "The rotation matrix of camera target pose of hole 4 relative to the base: \n" << target_pose_h4 << "\n" << endl;

        return 0;
    }
