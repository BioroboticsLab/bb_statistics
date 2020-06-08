#include "Settings.hpp"

#include "boost/program_options.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <sstream>

const boost::property_tree::ptree SettingsIAC::getDefaultParams()
{

    boost::property_tree::ptree pt;
    std::string                 app = SettingsIAC::setConf("");

    for (int i = 0; i < 4; i++)
    {
        boost::property_tree::ptree hd;
        hd.put(IMSTATISTICS::BUFFERCONF::CAMID, i);
        hd.put(IMSTATISTICS::BUFFERCONF::SERIAL, 0);
        hd.put(IMSTATISTICS::BUFFERCONF::ENABLED, 1);
        hd.put(IMSTATISTICS::BUFFERCONF::VIDEO_WIDTH, 4000);
        hd.put(IMSTATISTICS::BUFFERCONF::VIDEO_HEIGHT, 3000);
        pt.add_child(IMSTATISTICS::BUFFER, hd);
    }

    pt.put(IMSTATISTICS::DOANALYSIS, 1);
    pt.put(IMSTATISTICS::DOIMSAVE, 1);
    pt.put(IMSTATISTICS::SCALINGFACTOR, 2.0);
    pt.put(IMSTATISTICS::CRWINDOWSIZE, 800.0);
    pt.put(IMSTATISTICS::TARGET1, "/mnt/flip/cam0.jpg");
    pt.put(IMSTATISTICS::TARGET2, "/mnt/flip/cam1.jpg");
    pt.put(IMSTATISTICS::TARGET3, "/mnt/flip/cam2.jpg");
    pt.put(IMSTATISTICS::TARGET4, "/mnt/flip/cam3.jpg");
    pt.put(IMSTATISTICS::ANALYSISFILE, "analysis.txt");
    pt.put(IMSTATISTICS::ANALYSISFILEMIRROR, "analysisMirror.txt");

    return pt;
}

EncoderQualityConfig SettingsIAC::setFromNode(boost::property_tree::ptree node)
{
    EncoderQualityConfig cfg;

    cfg.camid   = node.get<int>(IMSTATISTICS::BUFFERCONF::CAMID);
    cfg.serial  = node.get<int>(IMSTATISTICS::BUFFERCONF::SERIAL);
    cfg.enabled = node.get<int>(IMSTATISTICS::BUFFERCONF::ENABLED);
    cfg.width   = node.get<int>(IMSTATISTICS::BUFFERCONF::VIDEO_WIDTH);
    cfg.height  = node.get<int>(IMSTATISTICS::BUFFERCONF::VIDEO_HEIGHT);

    return cfg;
}

EncoderQualityConfig SettingsIAC::getBufferConf(int camid)
{
    EncoderQualityConfig cfg;
    std::stringstream    cid;

    cfg.camid = -1;
    cid << camid;

    // Find the subtree having the right CAMID and ISPREVIEW values
    BOOST_FOREACH (boost::property_tree::ptree::value_type& v, _ptree.get_child("IMSTATISTICS"))
    {
        if (v.first == "BUFFER")
        {
            int hit = 0;
            BOOST_FOREACH (boost::property_tree::ptree::value_type& w, v.second)
            {
                std::string snd = w.second.data();
                if (w.first == "CAMID" && snd == cid.str())
                    hit++;

                if (hit == 1)
                {
                    return setFromNode(v.second);
                }
            }
        }
    }
    return cfg;
}
