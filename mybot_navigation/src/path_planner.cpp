#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include<math.h>
#include <vector>
#include "gnuplot-iostream.h"


class Coordinate{
   public:
	double x;
	double y;
    Coordinate(double x,double y){
        this->x = x;
        this->y = y;
    }
    Coordinate(){
        this->x = 0;
        this->y = 0;
    }
    void setX(double x){
        this->x = x;
    }
    void setY(double y){
        this->y = y;
    }
};


class CubicBezier{
    public:
        Coordinate P0,P1,P2,P3;
        double t = 0.0;
        double step_size = 0.01;
        std::vector<double> control_points_x;
        std::vector<double> control_points_y;
        std::vector<double> x_points;
        std::vector<double> y_points;
        double t3_coeff_x,t2_coeff_x,t_coeff_x,const_coeff_x;
        double t3_coeff_y,t2_coeff_y,t_coeff_y,const_coeff_y;
        double P0_coeff,P1_coeff,P2_coeff,P3_coeff;

        CubicBezier(Coordinate c1,Coordinate c2,Coordinate c3,Coordinate c4){
            this->P0 = Coordinate(c1.x,c1.y);
            this->P1 = Coordinate(c2.x,c2.y);
            this->P2 = Coordinate(c3.x,c3.y);
            this->P3 = Coordinate(c4.x,c4.y);
            this->control_points_x = {c1.x,c2.x,c3.x,c4.x};
            this->control_points_y = {c1.y,c2.y,c3.y,c4.y};
        }

        void createEquations(){
            // Equation Coefficiants for x
            this->t3_coeff_x = (-this->P0.x)+(3*this->P1.x)+(-3*this->P2.x)+(this->P3.x);
            this->t2_coeff_x = (3*this->P0.x)+(-6*this->P1.x)+(3*this->P2.x);
            this->t_coeff_x =  (-3*this->P0.x)+(3*this->P1.x);
            this->const_coeff_x = this->P0.x; 
            std::cout<<"("<<this->t3_coeff_x<<") t^3 + ("<<this->t2_coeff_x<<")+ t^2 +("<<this->t_coeff_x<<")+ t +("<<this->const_coeff_x<<")\n";

            // Equation Coefficiants for y
            this->t3_coeff_y = (-this->P0.y)+(3*this->P1.y)+(-3*this->P2.y)+(this->P3.y);
            this->t2_coeff_y = (3*this->P0.y)+(-6*this->P1.y)+(3*this->P2.y); 
            this->t_coeff_y =  (-3*this->P0.y)+(3*this->P1.y);
            this->const_coeff_y = this->P0.y; 
            std::cout<<"("<<this->t3_coeff_y<<") t^3 + ("<<this->t2_coeff_y<<")+ t^2 +("<<this->t_coeff_y<<")+ t +("<<this->const_coeff_y<<")\n";
        }

        void setCoeff(double t){
            this->P0_coeff = -pow(t,3)+3*pow(t,2)-3*t+1;
            this->P1_coeff = 3*pow(t,3)-6*pow(t,2)+3*t;
            this->P2_coeff = -3*pow(t,3)+3*pow(t,2);
            this->P3_coeff = pow(t,3);
        }

        double getXVal(double t){
            this->setCoeff(t);
            return (this->P0.x*this->P0_coeff)+(this->P1.x*this->P1_coeff)+(this->P2.x*this->P2_coeff)+(this->P3.x*this->P3_coeff);
        }
        double getYVal(double t){
            this->setCoeff(t);
            return (this->P0.y*this->P0_coeff)+(this->P1.y*this->P1_coeff)+(this->P2.y*this->P2_coeff)+(this->P3.y*this->P3_coeff);
    
        }

        Coordinate getPoint(double t){
            double x_cor = this->t3_coeff_x*pow(t,3)+this->t2_coeff_x*pow(t,2)+this->t_coeff_x*t+this->const_coeff_x;
            double y_cor = this->t3_coeff_y*pow(t,3)+this->t2_coeff_y*pow(t,2)+this->t_coeff_y*t+this->const_coeff_y;
            return Coordinate(x_cor,y_cor);
        }

        double getGradient(double t){
            double dx_t = (this->t3_coeff_x*3*pow(t,2))+(this->t2_coeff_x*2*t)+(this->t_coeff_x);
            double dy_t = (this->t3_coeff_y*3*pow(t,2))+(this->t2_coeff_y*2*t)+(this->t_coeff_y);
            if(dx_t!=0){
                return atan2(1,dy_t/dx_t)*180/3.14;
            }
            else{
                return 90.0;
            }
        }
        void findCurve(){
            while(this->t<=1){
                this->x_points.push_back(this->getXVal(this->t));
                this->y_points.push_back(this->getYVal(this->t));
                std::cout<<"for t = "<<this->t<<" the x : "<<this->getXVal(this->t)<<" and y : "<<this->getYVal(this->t)<<"\n";
                this->t+=this->step_size;
            }
        }

};

class CatmullRomBezierConverter:
    void CatmullRomBezierConverter(Coordinate c1,Coordinate c2,Coordinate c3,Coordinate c4){
		this->P0 = Coordinate(c1.x,c1.y);
		this->P1 = Coordinate(c2.x,c2.y);
		this->P2 = Coordinate(c3.x,c3.y);
		this->P3 = Coordinate(c4.x,c4.y);
		this->tou = 5;
        }
    double BezierToCatmull(){
		this->P0_prime = Coordinate(this->P3.x+6*(this->P0.x-this->P1.x),this->P3.y+6*(this->P0.y-this->P1.y));
		this->P1_prime = Coordinate(this->P0.x,this->P0.y);
		this->P2_prime = Coordinate(this->P3.x,this->P3.y);
		this->P3_prime = Coordinate(this->P0.x+6*(this->P3.x-this->P2.x),this->P0.y+6*(this->P3.y-this->P2.y));
		return this->P0_prime,this->P1_prime,this->P2_prime,this->P3_prime;
        }
    double CatmullToBezier(){
		this->P1_prime = Coordinate(this->P1.x+(this->P2.x-this->P0.x)/(this->tou*6),this->P1.y+(this->P2.y-this->P0.y)/(this->tou*6));
		this->P0_prime = Coordinate(this->P1.x,this->P1.y);
		this->P3_prime = Coordinate(this->P2.x,this->P2.y);
		this->P2_prime = Coordinate(this->P2.x-(this->P3.x-this->P1.x)/(this->tou*6),this->P2.y-(this->P3.y-this->P1.y)/(this->tou*6));
		return this->P0_prime,this->P1_prime,this->P2_prime,this->P3_prime;
	}
};

int main(){
    Coordinate c1(1,1);
    Coordinate c2(3,4);
    Coordinate c3(5,4);
    Coordinate c4(6,1);
    CubicBezier obj(c1,c2,c3,c4);
    obj.createEquations();
    obj.findCurve();
    std::cout<<"derivative at t = "<<0.2<<" is "<<obj.getGradient(0.2)<<"\n";
}
