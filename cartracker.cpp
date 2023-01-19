/* This is a modified file of the Simple Depth Viewer of the Orbbec Astra SDK [https://orbbec3d.com] used in compliance with the Apache License, Version 2.0 as a part of the Toyota Innovation Challenge hosted by the University of Waterloo.
 The goal of this hackathon was to modify this depth viewer in C++ to develop a vehicle tracking system and this is the submission for Group 17. Check out the final results of this code at https://bchharaw.github.io/#/projects/cardetection
This code requires an Astra SDK compatible depth camera, the Astra SDK, and the associated desktop drivers. The unmodified SDK and drivers can be found at https://drive.google.com/drive/folders/1ybOR5NLRfRJM09OUw7Nv3fQ8K34Sdg6A?usp=sharing
This code is meant to replace main.cpp in the following location of the SDK provided in the google drive \AstraSDKvs2015-win64-silas (1)\AstraSDKvs2015-win64-silas\samples\sfml\SimpleDepthViewer-SFML

*/
#include <SFML/Graphics.hpp>
#include <astra/astra.hpp>
#include "LitDepthVisualizer.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <key_handler.h>
#include <sstream>


class DepthFrameListener : public astra::FrameListener
{
public:

    DepthFrameListener()
    {
        prev_ = ClockType::now();
        font_.loadFromFile("Inconsolata.otf");
    }
    void screenshot_window(sf::RenderWindow& render_window, std::string filename)
    {
        sf::Texture texture;
        texture.create(render_window.getSize().x, render_window.getSize().y);
        texture.update(render_window);
        if (texture.copyToImage().saveToFile(filename))
            std::cout << "screenshot saved to" << filename << std::endl;
    }

    void init_texture(int width, int height)
    {
        if (!displayBuffer_ ||
            width != displayWidth_ ||
            height != displayHeight_)
        {
            displayWidth_ = width;
            displayHeight_ = height;

            // texture is RGBA
            const int byteLength = displayWidth_ * displayHeight_ * 4;

            displayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::fill(&displayBuffer_[0], &displayBuffer_[0] + byteLength, 0);

            texture_.create(displayWidth_, displayHeight_);
            sprite_.setTexture(texture_, true);
            sprite_.setPosition(0, 0);
        }
    }

    void check_fps()
    {
        const float frameWeight = .2f;

        const ClockType::time_point now = ClockType::now();
        const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

        elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
        prev_ = now;

        const float fps = 1000.f / elapsedMillis;

        const auto precision = std::cout.precision();

        /*std::cout << std::fixed
                  << std::setprecision(1)
                  << fps << " fps ("
                  << std::setprecision(1)
                  << elapsedMillis_ << " ms)"
                  << std::setprecision(precision)
                  << std::endl;
        */
    }

    void on_frame_ready(astra::StreamReader& reader,
        astra::Frame& frame) override
    {
        const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();
        const int width = pointFrame.width();
        const int height = pointFrame.height();

        init_texture(width, height);

        check_fps();

        if (isPaused_) { return; }

        copy_depth_data(frame);

        visualizer_.update(pointFrame);

        const astra::RgbPixel* vizBuffer = visualizer_.get_output();

        for (int i = 0; i < width * height; i++)
        {
            if (depthData_[i] > 1190 && depthData_[i] < 1280 && int(i / width) > 210 && int(i / width) < 255) {

                const int rgbaOffset = i * 4;
                displayBuffer_[rgbaOffset] = vizBuffer[i].r;
                displayBuffer_[rgbaOffset + 1] = vizBuffer[i].b;
                displayBuffer_[rgbaOffset + 2] = vizBuffer[i].g;
                displayBuffer_[rgbaOffset + 3] = 255;


            }
            else {
                const int rgbaOffset = i * 4;
                displayBuffer_[rgbaOffset] = 0;
                displayBuffer_[rgbaOffset + 1] = 0;
                displayBuffer_[rgbaOffset + 2] = 0;
                displayBuffer_[rgbaOffset + 3] = 0;
            }

        }

        texture_.update(displayBuffer_.get());

    }

    void copy_depth_data(astra::Frame& frame)
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

        if (depthFrame.is_valid())
        {
            const int width = depthFrame.width();
            const int height = depthFrame.height();
            if (!depthData_ || width != depthWidth_ || height != depthHeight_)
            {
                depthWidth_ = width;
                depthHeight_ = height;

                // texture is RGBA
                const int byteLength = depthWidth_ * depthHeight_ * sizeof(uint16_t);

                depthData_ = DepthPtr(new int16_t[byteLength]);
            }

            depthFrame.copy_to(&depthData_[0]);
        }
    }

    void update_mouse_position(sf::RenderWindow& window,
        const astra::CoordinateMapper& coordinateMapper)
    {
        const sf::Vector2i position = sf::Mouse::getPosition(window);
        const sf::Vector2u windowSize = window.getSize();

        float mouseNormX = position.x / float(windowSize.x);
        float mouseNormY = position.y / float(windowSize.y);

        mouseX_ = depthWidth_ * mouseNormX;
        mouseY_ = depthHeight_ * mouseNormY;

        if (mouseX_ >= depthWidth_ ||
            mouseY_ >= depthHeight_ ||
            mouseX_ < 0 ||
            mouseY_ < 0) {
            return;
        }

        const size_t index = (depthWidth_ * mouseY_ + mouseX_);
        const short z = depthData_[index];

        coordinateMapper.convert_depth_to_world(float(mouseX_),
            float(mouseY_),
            float(z),
            mouseWorldX_,
            mouseWorldY_,
            mouseWorldZ_);
    }

    void draw_text(sf::RenderWindow& window,
        sf::Text& text,
        sf::Color color,
        const int x,
        const int y) const
    {
        text.setColor(sf::Color::Black);
        text.setPosition(x + 5, y + 5);
        window.draw(text);

        text.setColor(color);
        text.setPosition(x, y);
        window.draw(text);
    }

    void draw_mouse_overlay(sf::RenderWindow& window,
        const float depthWScale,
        const float depthHScale) const
    {
        if (!isMouseOverlayEnabled_ || !depthData_) { return; }

        std::stringstream str;
        str << std::fixed
            << std::setprecision(0)
            << "(" << mouseX_ << ", " << mouseY_ << ") "
            << "X: " << mouseWorldX_ << " Y: " << mouseWorldY_ << " Z: " << mouseWorldZ_;

        const int characterSize = 40;
        sf::Text text(str.str(), font_);
        text.setCharacterSize(characterSize);
        text.setStyle(sf::Text::Bold);

        const float displayX = 10.f;
        const float margin = 10.f;
        const float displayY = window.getView().getSize().y - (margin + characterSize);

        draw_text(window, text, sf::Color::White, displayX, displayY);
    }

    void draw_to(sf::RenderWindow& window)
    {

        if (displayBuffer_ != nullptr)
        {
            const float depthWScale = window.getView().getSize().x / displayWidth_;
            const float depthHScale = window.getView().getSize().y / displayHeight_;

            sprite_.setScale(depthWScale, depthHScale);
            window.draw(sprite_);

            draw_mouse_overlay(window, depthWScale, depthHScale);
        }
    }
    void draw_rec(sf::RenderWindow& window, sf::RectangleShape& rectangle, sf::RectangleShape& wheelrec)
    {


        bool first_pic = true;
        if (displayBuffer_ != nullptr)
        {
            int diameter = 15;
            int top = 0;
            int bottom = 0;
            int left = 0;
            int right = 0;
            int lastx = 0;
            int lasty = 0;
            const float depthWScale = window.getView().getSize().x / displayWidth_;
            const float depthHScale = window.getView().getSize().y / displayHeight_;

            for (int i = 0; i < displayWidth_ * displayHeight_; i++)
            {
                const int rgbaOffset = i * 4;
                if (displayBuffer_[rgbaOffset] != 0 && displayBuffer_[rgbaOffset + 1] != 0 && displayBuffer_[rgbaOffset + 2] != 0 && displayBuffer_[rgbaOffset + 3] != 0 && first_pic == true) {
                    first_pic = false;
                    top = i / double(displayWidth_);
                    left = i;
                    right = i % displayWidth_;
                }
                if (displayBuffer_[rgbaOffset] != 0 && displayBuffer_[rgbaOffset + 1] != 0 && displayBuffer_[rgbaOffset + 2] != 0 && displayBuffer_[rgbaOffset + 3] != 0) {
                    bottom = i / double(displayWidth_);

                    if ((i % displayWidth_) < left) {
                        left = i % displayWidth_;
                    }
                    if ((i % displayWidth_) > right) {
                        right = i % displayWidth_;
                    }


                }

            }

            for (int i = 0; i < displayWidth_ * displayHeight_; i++)
            {
                const int rgbaOffset = i * 4;

                if (displayBuffer_[rgbaOffset] != 0 && displayBuffer_[rgbaOffset + 1] != 0 && displayBuffer_[rgbaOffset + 2] != 0 && displayBuffer_[rgbaOffset + 3] != 0 && i > ((right - left) / 2)) {
                    lastx = i % displayWidth_;

                }
            }




            std::cout << "T: " << top << " B: " << bottom << " L: " << left << " R: " << right << std::endl;


            wheelrec.setSize(sf::Vector2f(diameter * 2, diameter * 2));
            wheelrec.setPosition((lastx - (diameter / 2)) * 2, (bottom - diameter) * 2);


            rectangle.setSize(sf::Vector2f((right - left) * 2, (bottom - top) * 2));
            rectangle.setPosition(2 * left, 2 * top);
            window.draw(rectangle);


        }

    }
    void toggle_paused()
    {
        isPaused_ = !isPaused_;
    }

    bool is_paused() const
    {
        return isPaused_;
    }

    void toggle_overlay()
    {
        isMouseOverlayEnabled_ = !isMouseOverlayEnabled_;
    }

    bool overlay_enabled() const
    {
        return isMouseOverlayEnabled_;
    }

private:
    samples::common::LitDepthVisualizer visualizer_;

    using DurationType = std::chrono::milliseconds;
    using ClockType = std::chrono::high_resolution_clock;

    ClockType::time_point prev_;
    float elapsedMillis_{ .0f };

    sf::Texture texture_;
    sf::Sprite sprite_;
    sf::Font font_;

    int displayWidth_{ 0 };
    int displayHeight_{ 0 };

    using BufferPtr = std::unique_ptr<uint8_t[]>;
    BufferPtr displayBuffer_{ nullptr };

    int depthWidth_{ 0 };
    int depthHeight_{ 0 };

    using DepthPtr = std::unique_ptr<int16_t[]>;
    DepthPtr depthData_{ nullptr };

    int mouseX_{ 0 };
    int mouseY_{ 0 };
    float mouseWorldX_{ 0 };
    float mouseWorldY_{ 0 };
    float mouseWorldZ_{ 0 };
    bool isPaused_{ false };
    bool isMouseOverlayEnabled_{ true };
};

astra::DepthStream configure_depth(astra::StreamReader& reader)
{
    auto depthStream = reader.stream<astra::DepthStream>();

    auto oldMode = depthStream.mode();

    //We don't have to set the mode to start the stream, but if you want to here is how:
    astra::ImageStreamMode depthMode;

    depthMode.set_width(640);
    depthMode.set_height(480);
    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
    depthMode.set_fps(30);

    depthStream.set_mode(depthMode);

    auto newMode = depthStream.mode();
    printf("Changed depth mode: %dx%d @ %d -> %dx%d @ %d\n",
        oldMode.width(), oldMode.height(), oldMode.fps(),
        newMode.width(), newMode.height(), newMode.fps());

    return depthStream;
}

int main(int argc, char** argv)
{
    sf::RectangleShape wheelrec;
    wheelrec.setOutlineColor(sf::Color(255, 0, 0, 255));

    wheelrec.setOutlineThickness(5);
    wheelrec.setFillColor(sf::Color(0, 0, 0, 0));
    sf::RectangleShape rectangle;
    rectangle.setOutlineColor(sf::Color(95, 0, 160, 255));

    rectangle.setOutlineThickness(5);
    rectangle.setFillColor(sf::Color(0, 0, 0, 0));


    astra::initialize();

    set_key_handler();

    sf::RenderWindow window(sf::VideoMode(1280, 960), "Depth Viewer");

#ifdef _WIN32
    auto fullscreenStyle = sf::Style::None;
#else
    auto fullscreenStyle = sf::Style::Fullscreen;
#endif

    const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
    const sf::VideoMode windowedMode(1280, 1024);
    bool isFullScreen = false;

    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();
    reader.stream<astra::PointStream>().start();

    auto depthStream = configure_depth(reader);
    depthStream.start();

    DepthFrameListener listener;

    reader.add_listener(listener);

    while (window.isOpen())
    {
        if ((wheelrec.getPosition().x + 7.5) == 600) {
            listener.screenshot_window(window, "toyota.png");
            std::cout << "took ss";
        }
        astra_update();

        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            case sf::Event::KeyPressed:
            {
                if (event.key.code == sf::Keyboard::C && event.key.control)
                {
                    window.close();
                }
                switch (event.key.code)
                {
                case sf::Keyboard::D:
                {
                    auto oldMode = depthStream.mode();
                    astra::ImageStreamMode depthMode;

                    depthMode.set_width(640);
                    depthMode.set_height(400);
                    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
                    depthMode.set_fps(30);

                    depthStream.set_mode(depthMode);
                    auto newMode = depthStream.mode();
                    printf("Changed depth mode: %dx%d @ %d -> %dx%d @ %d\n",
                        oldMode.width(), oldMode.height(), oldMode.fps(),
                        newMode.width(), newMode.height(), newMode.fps());
                    break;
                }
                case sf::Keyboard::Escape:
                    window.close();
                    break;
                case sf::Keyboard::F:
                    if (isFullScreen)
                    {
                        window.create(windowedMode, "Depth Viewer", sf::Style::Default);
                    }
                    else
                    {
                        window.create(fullScreenMode, "Depth Viewer", fullscreenStyle);
                    }
                    isFullScreen = !isFullScreen;
                    break;
                case sf::Keyboard::R:
                    depthStream.enable_registration(!depthStream.registration_enabled());
                    break;
                case sf::Keyboard::M:
                    depthStream.enable_mirroring(!depthStream.mirroring_enabled());
                    break;
                case sf::Keyboard::P:
                    listener.toggle_paused();
                    break;
                case sf::Keyboard::Space:
                    listener.toggle_overlay();
                    break;
                default:
                    break;
                }
                break;
            }
            case sf::Event::MouseMoved:
            {
                auto coordinateMapper = depthStream.coordinateMapper();
                listener.update_mouse_position(window, coordinateMapper);

                window.clear(sf::Color::Black);

                listener.draw_to(window);
                window.display();
                break;
            }
            default:
                break;
            }
        }
        // clear the window with black color
        window.clear(sf::Color::Black);

        listener.draw_rec(window, rectangle, wheelrec);

        listener.draw_to(window);
        window.draw(wheelrec);
        window.display();


        if (!shouldContinue)
        {

            window.close();
        }
    }

    astra::terminate();

    return 0;
}

