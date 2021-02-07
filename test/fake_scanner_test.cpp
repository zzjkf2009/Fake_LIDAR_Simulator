/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-09-28T10:23:13-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: fake_scanner_test.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-09-28T10:23:59-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */
 #include <gtest/gtest.h>
 #include "fake_scanner.h"



TEST(fake_scannerTest, ConstructorTest)
{
        std::string topicName("fake_scan");
        Scan scan(100, 1.2, -3.14, 3.14, 0, 6.2,topicName);
        EXPECT_EQ(100, scan.getNum());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
        ros::init(argc, argv, "fake_scanner_test");
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
