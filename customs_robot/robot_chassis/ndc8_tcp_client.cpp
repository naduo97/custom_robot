#include "ndc8_tcp_client.h"
#include <QDebug>
#include <string.h>
//#include <libxml/parser.h>
//#include <libxml/tree.h>

namespace RobotChassis{
NDC8TcpClient::NDC8TcpClient(NDC8_DATA_CALLBACK callback)
{
    pDataCallback_ = callback;
}

NDC8TcpClient::~NDC8TcpClient()
{

}

void NDC8TcpClient::ParseXmlData(xmlNodePtr node)
{
    if (node == NULL)
    {
        return;
    }
//    qDebug() << (char*)node->name;
    xmlAttrPtr attr = node->properties;
    while (attr != NULL)
    {
        xmlChar* attr_name = (xmlChar*)attr->name;
        xmlChar* attr_value = xmlGetProp(node, attr_name);
//        qDebug() << (char*)attr_name << " -- " << (char*)attr_value;
        xmlChar* value = xmlNodeGetContent(node);
        if (value != NULL && std::string((char*)node->name) == "Item")
        {
//            qDebug() << (char*)value;
            if (std::string((char*)attr->name) == "Tag" && nullptr != pDataCallback_)
            {
                pDataCallback_(std::string((char*)attr_name), std::string((char*)attr_value), std::string((char*)value));
            }
            else
            {
                pDataCallback_(std::string((char*)attr_name), std::string((char*)attr_value), std::string(""));
            }
            xmlFree(value);
        }
        else
        {
            pDataCallback_(std::string((char*)attr_name), std::string((char*)attr_value), std::string(""));
        }
        xmlFree(attr_value);
        attr = attr->next;
    }
    xmlNodePtr child = node->children;
    while (child != NULL)
    {
        ParseXmlData(child);
        child = child->next;
    }
}

// 消息处理
void NDC8TcpClient::DataProcessing(const std::string &data)
{
    // 消息处理
    xmlDocPtr pXmlDP = xmlReadMemory(data.c_str(), data.size(), NULL, NULL, 0);
    xmlNodePtr pXmlNode = xmlDocGetRootElement(pXmlDP);
    ParseXmlData(pXmlNode);
    xmlFree(pXmlNode);
    xmlFree(pXmlDP);
    xmlCleanupParser();

//    ChassisStateData TempData;
//    if (nullptr != pDataCallback_)
//    {
//        pDataCallback_(TempData);
//    }
}

// 消息验证
bool NDC8TcpClient::CheckValidData(std::string &data, int &valid_start, int &valid_size)
{
    int iHeadIndex = -1;
    int iEndIndex = -1;
    iHeadIndex = data.find("<CPI2");
    if (iHeadIndex > -1)
    {   // 找到了
        //判断是不是 报错的消息
        if (iHeadIndex + 7 > (int)data.size())
        {
            valid_start = 0;
            valid_size = 0;
            return false;
        }
        if (data.c_str()[iHeadIndex + 5] == '>')
        {
            iEndIndex = data.find("</CPI2>", iHeadIndex + 5);
            if (iEndIndex > -1)
            {
//                qDebug() << data.substr(iHeadIndex, iEndIndex + 7 - iHeadIndex).c_str();
                valid_start = iHeadIndex;
                valid_size = iEndIndex - iHeadIndex + 7;
                return true;
            }
        }
        else
        {   // 极大可能是个报错的消息
            iEndIndex = data.find("/>", iHeadIndex + 4);
            if (iEndIndex > -1)
            {
//                qDebug() << data.substr(iHeadIndex, iEndIndex + 2 - iHeadIndex).c_str();
                valid_start = iHeadIndex;
                valid_size = iEndIndex - iHeadIndex + 2;
                return true;
            }
        }
    }
    valid_start = 0;
    valid_size = 0;
    return false;
}

}
