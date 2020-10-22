#include "FichierPoint.h"

using namespace std;
namespace pt = boost::property_tree;

vector<Point> FichierPoint::readPoints(string filename) {
    vector<Point> pts;

    string type;
    float X, Y, angle;
    float distance_tolerance;
    float angle_tolerance;
    int speed;
    bool slipping;
    bool detection;
    int score;
    //string sens, blocage;
    //double coefCourbe;
    //bool lissage, derapage, attAction;
    int timeout;
    string action;
    Point::Trajectory mTrajectory;

    cout << "Lecture du fichier point : '" << filename << "'" << endl;

    pt::ptree liste;
    pt::read_json(filename, liste);

    BOOST_FOREACH(const pt::ptree::value_type &point, liste.get_child("Points")) {
        X = 0;
        Y = 0;
        angle = 0;
        distance_tolerance = 0.0;
        angle_tolerance = 0.0;
        speed = 0;
        timeout = 0;
        action = "null";
        slipping = false;
        detection = false;
        score = 0;
        


        type = point.second.get<string>("type");

        if(type.compare("XY_ABSOLU") == 0) {
            X = point.second.get<float>("X");
            Y = point.second.get<float>("Y");
            //angle = point.second.get<float>("THETA");
            distance_tolerance = point.second.get<float>("distance_tolerance");
            angle_tolerance = point.second.get<float>("angle_tolerance");
            speed = point.second.get<int>("speed");
            timeout = point.second.get<int>("timeout");
            action = point.second.get<string>("action");
            slipping = point.second.get<bool>("slipping");
            detection = point.second.get<bool>("detection");
            score = point.second.get<int>("score");
            mTrajectory = Point::Trajectory::XY_ABSOLU;

        }
        else if (type.compare("THETA") == 0) {
            angle = point.second.get<float>("THETA");
            angle_tolerance = point.second.get<float>("angle_tolerance");
            speed = point.second.get<int>("speed");
            timeout = point.second.get<int>("timeout");
            action = point.second.get<string>("action");
            detection = point.second.get<bool>("detection");
            score = point.second.get<int>("score");
            mTrajectory = Point::Trajectory::THETA;
        }
        else if(type.compare("POSITION") == 0) {
            X = point.second.get<float>("X");
            Y = point.second.get<float>("Y");
            angle = point.second.get<float>("THETA");
            mTrajectory = Point::Trajectory::NOTHING;
        }
        else if(type.compare("CALIB_X") == 0) {
            X = point.second.get<float>("X");
            angle = point.second.get<float>("THETA");
            mTrajectory = Point::Trajectory::CALIB_X;
        }
        else if(type.compare("CALIB_Y") == 0) {
            Y = point.second.get<float>("Y");
            angle = point.second.get<float>("THETA");
            mTrajectory = Point::Trajectory::CALIB_Y;
        }
        else if(type.compare("CALIB_XY") == 0) {
            X = point.second.get<float>("X");
            Y = point.second.get<float>("Y");
            angle = point.second.get<float>("THETA");
            mTrajectory = Point::Trajectory::CALIB_XY;
        }
        else {
            continue;//aucun type connu, on passe à l'itération suivante
        }



        // Controller
        Point p(X, Y, angle, mTrajectory);
        p.setMAction(ref(action));
        p.setSlipping(slipping);

        //Detection et Score
        p.setLDetection(detection);
        p.setScore(score);

        //Point p(type, X, Y, angle, deltaDeplacement, deltaAngle, vitesse, sens, blocage, coefCourbe, lissage, derapage, timeOut, action, attAction);
        pts.push_back(p);

    }

    // point pour locker
    Point pblocked(0,0,0, Point::Trajectory::LOCKED);
    pts.push_back(pblocked);

    return pts;
}