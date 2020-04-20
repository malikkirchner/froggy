/*====================================================================================================================*/
/*                                                                                                                    */
/*      FROGGY     Copyright ©2020 by  Malik Kirchner <kirchner@xelonic.com>                                          */
/*                                                                                                                    */
/*      This program is free software: you can redistribute it and/or modify                                          */
/*      it under the terms of the GNU General Public License as published by                                          */
/*      the Free Software Foundation, either version 3 of the License, or                                             */
/*      (at your option) any later version.                                                                           */
/*                                                                                                                    */
/*      This program is distributed in the hope that it will be useful,                                               */
/*      but WITHOUT ANY WARRANTY; without even the implied warranty of                                                */
/*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                                 */
/*      GNU General Public License for more details.                                                                  */
/*                                                                                                                    */
/*      You should have received a copy of the GNU General Public License                                             */
/*      along with this program.  If not, see <https://www.gnu.org/licenses/>.                                        */
/*                                                                                                                    */
/*====================================================================================================================*/

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef USE_GTK
#include <opencv2/highgui.hpp>
#endif

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include <array>
#include <cmath>
#include <iostream>


namespace {

// Gravitational constant
constexpr double G = 6.6743015e-11;    // m^3/(kg*s^2)

}    // namespace

struct Coordinates {
    double          t = 0.;
    Eigen::Vector2d position;
    Eigen::Vector2d speed;
};

struct Body {
    Eigen::Vector2d position = { 0., 0. };    // meter from origin
    Eigen::Vector2d speed    = { 0., 0. };    // meter/second
    double          mass     = 0.;            // kilogram
    double          radius   = 0.;            // meter
};

struct Stage {
    double thrust   = 0.;    // Newton
    double duration = 0.;    // seconds
    double mass     = 0.;    // kilogram
};


struct Scene {
    Body rocket;
    Body earth;
    Body moon;

    std::vector< Stage > stages;

    double        t                   = 0.;
    double        dt                  = 0.01;
    std::uint64_t k                   = 0;
    double        initial_rocket_mass = 0.;

    double        zoom   = 1.;
    std::uint16_t width  = 1500;
    std::uint16_t height = 1500;
    std::uint16_t k_step = 100;

    std::vector< Coordinates > rocket_trajectory;
    std::vector< Coordinates > earth_trajectory;
    std::vector< Coordinates > moon_trajectory;
};

Eigen::Vector2d gravitational_force( const Body& a, const Body& b, const Body& c ) {
    const auto r_b = b.position - a.position;
    const auto r_c = c.position - a.position;

    return G * ( r_b.normalized() * b.mass / r_b.squaredNorm() + r_c.normalized() * c.mass / r_c.squaredNorm() );
}

Eigen::Vector2d thrust( const Scene& scene ) {
    double k = 0.;
    double m = scene.initial_rocket_mass;
    for ( const auto& stage : scene.stages ) {
        const auto a = m - stage.mass / stage.duration * ( scene.t - k );
        k += stage.duration;
        m -= stage.mass;
        if ( scene.t < k ) {
            const Eigen::Rotation2Dd rot{ M_PI / 360. * 37.1 };
            const auto               r = scene.moon.position - scene.rocket.position;
            return stage.thrust / a * ( rot * r.normalized() );
        }
    }

    return { 0., 0. };
}


bool leap_frog( Scene& scene ) {
    auto rocket = scene.rocket;
    auto earth  = scene.earth;
    auto moon   = scene.moon;

    {
        const auto acc0 = gravitational_force( scene.rocket, scene.earth, scene.moon ) + 12.803 * thrust( scene );

        const auto v = rocket.speed + acc0 * 0.5 * scene.dt;
        const auto x = rocket.position + v * 0.5 * scene.dt;

        rocket.position = x + v * 0.5 * scene.dt;
        const auto acc1 = gravitational_force( rocket, scene.earth, scene.moon ) + 12.803 * thrust( scene );
        rocket.speed    = v + acc1 * 0.5 * scene.dt;

        // std::cout << acc0 << std::endl;
    }


    {
        const auto acc0 = gravitational_force( scene.moon, scene.earth, scene.rocket );
        const auto v    = moon.speed + acc0 * 0.5 * scene.dt;
        const auto x    = moon.position + v * 0.5 * scene.dt;

        moon.position   = x + v * 0.5 * scene.dt;
        const auto acc1 = gravitational_force( moon, scene.earth, scene.rocket );
        moon.speed      = v + acc1 * 0.5 * scene.dt;
    }

    {
        const auto acc0 = gravitational_force( scene.earth, scene.moon, scene.rocket );
        const auto v    = earth.speed + acc0 * 0.5 * scene.dt;
        const auto x    = earth.position + v * 0.5 * scene.dt;

        earth.position  = x + v * 0.5 * scene.dt;
        const auto acc1 = gravitational_force( earth, scene.moon, scene.rocket );
        earth.speed     = v + acc1 * 0.5 * scene.dt;
    }

    scene.rocket = rocket;
    scene.earth  = earth;
    scene.moon   = moon;
    scene.t += scene.dt;
    ++scene.k;

    if ( scene.k % scene.k_step == 0 ) {
        scene.rocket_trajectory.emplace_back( Coordinates{ scene.t, scene.rocket.position, scene.rocket.speed } );
        scene.earth_trajectory.emplace_back( Coordinates{ scene.t, scene.earth.position, scene.earth.speed } );
        scene.moon_trajectory.emplace_back( Coordinates{ scene.t, scene.moon.position, scene.moon.speed } );
    }

    return ( scene.rocket.position - scene.earth.position ).norm() >= scene.earth.radius;
}


cv::Mat draw( const Scene& scene ) {
    cv::Mat buffer( scene.width, scene.height, CV_32FC3 );

    {
        const int x = 0.5 * scene.width + std::round( scene.earth.position.x() * scene.zoom );
        const int y = 0.5 * scene.height + std::round( scene.earth.position.y() * scene.zoom );
        const int r = std::max< int >( std::roundl( scene.earth.radius * scene.zoom ), 1 );
        cv::circle( buffer, cv::Point( x, y ), r, cv::Scalar( 1., 0.2, 0.2 ), cv::FILLED, 8, 0 );
        spdlog::info( "Earth ({},{}|{})", x, y, r );
    }

    {
        const int x = 0.5 * scene.width + std::round( scene.moon.position.x() * scene.zoom );
        const int y = 0.5 * scene.height + std::round( scene.moon.position.y() * scene.zoom );
        const int r = std::max< int >( std::roundl( scene.moon.radius * scene.zoom ), 1 );
        cv::circle( buffer, cv::Point( x, y ), r, cv::Scalar( .8, .8, .8 ), cv::FILLED, 8, 0 );
        spdlog::info( "Moon ({},{}|{})", x, y, r );
    }

    {
        const int x = 0.5 * scene.width + std::round( scene.rocket.position.x() * scene.zoom );
        const int y = 0.5 * scene.height + std::round( scene.rocket.position.y() * scene.zoom );
        const int r = std::max< int >( std::roundl( scene.rocket.radius * scene.zoom ), 5 );
        cv::circle( buffer, cv::Point( x, y ), r, cv::Scalar( 0., .4, 1. ), cv::FILLED, 8, 0 );
        spdlog::info( "Rocket ({},{}|{})", x, y, r );
    }

    const unsigned step = 1;

    {
        for ( unsigned k = step; k < scene.rocket_trajectory.size(); k += step ) {
            const int x0 =
                    0.5 * scene.width + std::round( scene.rocket_trajectory[ k - step ].position.x() * scene.zoom );
            const int y0 =
                    0.5 * scene.height + std::round( scene.rocket_trajectory[ k - step ].position.y() * scene.zoom );
            const int x1 = 0.5 * scene.width + std::round( scene.rocket_trajectory[ k ].position.x() * scene.zoom );
            const int y1 = 0.5 * scene.height + std::round( scene.rocket_trajectory[ k ].position.y() * scene.zoom );
            cv::line( buffer, cv::Point2d( x0, y0 ), cv::Point2d( x1, y1 ), cv::Scalar( 0.0, 0.4, 1.0 ) );
        }
    }

    {
        for ( unsigned k = step; k < scene.earth_trajectory.size(); k += step ) {
            const int x0 =
                    0.5 * scene.width + std::round( scene.earth_trajectory[ k - step ].position.x() * scene.zoom );
            const int y0 =
                    0.5 * scene.height + std::round( scene.earth_trajectory[ k - step ].position.y() * scene.zoom );
            const int x1 = 0.5 * scene.width + std::round( scene.earth_trajectory[ k ].position.x() * scene.zoom );
            const int y1 = 0.5 * scene.height + std::round( scene.earth_trajectory[ k ].position.y() * scene.zoom );
            cv::line( buffer, cv::Point2d( x0, y0 ), cv::Point2d( x1, y1 ), cv::Scalar( 0.5, 0.2, 0.5 ) );
        }
    }

    {
        for ( unsigned k = step; k < scene.moon_trajectory.size(); k += step ) {
            const int x0 =
                    0.5 * scene.width + std::round( scene.moon_trajectory[ k - step ].position.x() * scene.zoom );
            const int y0 =
                    0.5 * scene.height + std::round( scene.moon_trajectory[ k - step ].position.y() * scene.zoom );
            const int x1 = 0.5 * scene.width + std::round( scene.moon_trajectory[ k ].position.x() * scene.zoom );
            const int y1 = 0.5 * scene.height + std::round( scene.moon_trajectory[ k ].position.y() * scene.zoom );
            cv::line( buffer, cv::Point2d( x0, y0 ), cv::Point2d( x1, y1 ), cv::Scalar( 0.5, 0.5, 0.5 ) );
        }
    }

    return buffer;
}


// https://en.wikipedia.org/wiki/Orbit_of_the_Moon#/media/File:Lunar_Orbit_and_Orientation_with_respect_to_the_Ecliptic.tif
// https://de.wikipedia.org/wiki/Ariane_5
int main() {
    const double d_earth_moon = 384'400'000.;

    Scene scene;
    scene.earth.mass    = 5.9722e24;
    scene.earth.radius  = 6'371'000.;
    scene.moon.mass     = 7.34767309e22;
    scene.moon.radius   = 1'737'100.;
    scene.rocket.mass   = 750'000;
    scene.rocket.radius = 27.;

    const double M       = scene.earth.mass + scene.moon.mass;
    scene.earth.position = { -d_earth_moon * scene.moon.mass / M, 0. };
    scene.moon.position  = { d_earth_moon * scene.earth.mass / M, 0. };
    scene.earth.speed    = { 0., -2. * M_PI * std::abs( scene.earth.position.x() ) / ( 29 * 24 * 3600. ) };
    scene.moon.speed     = { 0., +2. * M_PI * std::abs( scene.moon.position.x() ) / ( 29 * 24 * 3600. ) };

    scene.rocket.position = { scene.earth.position.x() + scene.earth.radius, scene.earth.position.y() };
    scene.rocket.speed    = { scene.earth.speed.x(),
                           scene.earth.speed.y() + 2. * M_PI * scene.earth.radius / ( 24. * 3600. ) };

    scene.rocket_trajectory.emplace_back( Coordinates{ scene.t, scene.rocket.position, scene.rocket.speed } );
    scene.earth_trajectory.emplace_back( Coordinates{ scene.t, scene.earth.position, scene.earth.speed } );
    scene.moon_trajectory.emplace_back( Coordinates{ scene.t, scene.moon.position, scene.moon.speed } );

    Stage booster_stage;
    booster_stage.thrust   = 4'000'000;
    booster_stage.duration = 130.;
    booster_stage.mass     = 270'000;

    Stage main_stage;
    main_stage.thrust   = 1'180'000;
    main_stage.duration = 605.;
    main_stage.mass     = 170'500;

    Stage upper_stage;
    main_stage.thrust   = 27'000;
    main_stage.duration = 1100.;
    main_stage.mass     = 10'900;

    scene.stages.push_back( std::move( booster_stage ) );
    scene.stages.push_back( std::move( main_stage ) );
    scene.stages.push_back( std::move( upper_stage ) );

    scene.zoom = 0.3 * std::sqrt( scene.width * scene.width + scene.height * scene.height ) / d_earth_moon;
    scene.initial_rocket_mass = scene.rocket.mass;

#ifdef USE_GTK
    cv::namedWindow( "froggy", cv::WINDOW_AUTOSIZE );
    cv::moveWindow( "froggy", 0, 0 );
#endif

    const double T = 10 * 24 * 3600;
    scene.rocket_trajectory.reserve( T / scene.dt / scene.k_step + 100 );
    scene.earth_trajectory.reserve( T / scene.dt / scene.k_step + 100 );
    scene.moon_trajectory.reserve( T / scene.dt / scene.k_step + 100 );
    for ( scene.t = 0; scene.t < T; ) {
        if ( !leap_frog( scene ) ) {
            break;
        };
    }

    const cv::Mat buffer = draw( scene );
#ifdef USE_GTK
    cv::imshow( "froggy", buffer );
    cv::waitKey( 0 );
#endif

    return 0;
}