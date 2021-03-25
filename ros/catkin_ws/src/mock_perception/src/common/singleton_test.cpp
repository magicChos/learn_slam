//
// Created by dyq on 2020/11/17.
//

#include "ace/common/singleton.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

namespace ace {
namespace common {

class DataTest
{
public:
    DataTest() : data_(0) {}

    DataTest(int data) : data_(data) {}

    int GetData() const
    {
        return data_;
    }

    void SetData(int data)
    {
        data_ = data;
    }

private:
    int data_;
};


TEST(Singleton, test02)
{
    DataTest* data = Singleton<DataTest>::GetInstance(100).get();
    LOG(INFO) << "data: " << data->GetData() ;
}

TEST(Singleton, test01)
{
    DataTest* data = Singleton<DataTest>::GetInstance().get();
    LOG(INFO) << "data: " << data->GetData() ;

    data->SetData(123);
    LOG(INFO) << "data: " << data->GetData() ;
}



} // namespace common
} // namespace ace
