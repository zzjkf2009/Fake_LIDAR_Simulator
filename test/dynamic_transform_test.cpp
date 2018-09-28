/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-28T09:41:39-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: dynamic_transform_test.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-28T09:42:04-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */
 #include <gtest/gtest.h>
 #include "../include/fake_scanner/dynamic_transform.h"
/*
   struct dynamicTest : public ::testing::Test {
        Dynamic_Transform dynamic_transform(0.1,0.2,0.3,0.4,0.01,0.02,0.03);
   };*/
// Declare a test
TEST(dynamicTest, ConstructorTest)
{
        Dynamic_Transform dynamic_transform(0.1,0.2,0.3,0.4,0.01,0.02,0.03);
        EXPECT_EQ(0.02, dynamic_transform.getY_incremental());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "dynamic_transform_tester");
        return RUN_ALL_TESTS();
}
