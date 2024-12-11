#include <qp/control_tree_plot.h>
#include <iostream>

void plot(const std::function<double(int i)> & value_provider,
          const std::vector<IntA> & varss,
          const std::vector<Arr> & scaless,
          const PlotAxis & axis)
{
    TreePlot plotter(varss.size(), axis.name, axis.range);
    plotter.update(varss, scaless, value_provider);
}

void save(const std::function<double(int i)> & value_provider,
          double dt,
          const std::vector<IntA> & varss,
          const std::vector<Arr> & scales,
          std::ostream & file)
{
    for(const auto & vars : varss)
    {
        save(value_provider, dt, vars, file);
    }

    file << std::endl;
}


void save(const std::function<double(int i)> & value_provider,
          double dt,
          const IntA & vars, std::ostream & file)
{
    for(uint i=0; i<vars.size(); i+=1)
    {
        auto x = i * dt;
        auto y = value_provider(vars[i]);
        file << "  " << x << " " << y << std::endl;
    }

    file << std::endl;
    file << std::endl;
}

void TreePlot::update(const std::vector<IntA> & varss,
            const std::vector<Arr> & scales,
            const std::function<double(int i)> & value_provider)
{
    auto color_code = [](int i, double p)->std::string
    {
        std::stringstream ss;
        ss << "#";
        ss << std::hex << int((1.0 - sqrt(p)) * 255);
        ss << "00";
        ss << "00";
        ss << "FF";

        return ss.str();
    };

    auto & gp = gp_;

    if(!window_title_set_)
    {
      //gp << "set term wxt title 'Trajectory-tree display'\n";
      gp << "set term wxt title 'Trajectory-tree display' size 400,350 position 432," << (name_ == "acceleration" ? "125" : "551") << "\n";
      window_title_set_ = true;
    }
    gp << "set title '" << name_ << "'\n";
    gp << "set xrange [0:19]\nset yrange " << yrange_ << "\n";
    gp << "set xtics ('1.0' 3, '2.0' 7, '3.0' 11, '4.0' 15, '5.0' 19)\n";
    gp << "plot ";

    for(uint i = 0; i < varss.size(); ++i)
    {
        gp << "'-' with lines title '" << name_ << "-" << i << "'";
        gp << " lc rgb '"<< color_code(i, scales[i].back()) <<"'";

        if(i < varss.size() - 1)
            gp << ",";
    }

    gp << "\n";


    // data
    std::vector<std::vector<std::pair<double, double> > > xy_pts_all;

    for(const auto& vars : varss)
    {
        std::vector<std::pair<double, double> > xy_pts;
        for(uint i=0; i<vars.size(); i+=1)
        {
            double x = i;
            double y = value_provider(vars[i]);
            xy_pts.push_back(std::make_pair(x, y));
        }
        xy_pts_all.push_back(xy_pts);
    }


    for(auto xy_pts : xy_pts_all)
    {
        gp.send1d(xy_pts);
    }
}
