#pragma once

#include "../Robot.hh"
#include "Drawable.hh"

class GL {
public:
  SDL_Window* gWindow = NULL; //The window we'll be rendering to
  SDL_GLContext gContext; //OpenGL context
  SDL_Renderer* renderer = NULL;

  TTF_Font* font;
  SDL_Color textColor = { 255, 255, 255, 255 }; // white
  SDL_Color backgroundColor = { 0, 0, 0, 255 }; // black

  SDL_Texture* solidTexture;
  SDL_Texture* blendedTexture;
  SDL_Texture* shadedTexture;

  SDL_Rect solidRect;
  SDL_Rect blendedRect;
  SDL_Rect shadedRect;
  SDL_Rect windowRect = { 900, 300, 400, 400 };

  bool simulate = false;
  bool zoom = true;

  int SCREEN_WIDTH;
  int SCREEN_HEIGHT;

  Robot* robot;

  std::vector<Drawable*> drawables;

  GL(int _SCREEN_WIDTH, int _SCREEN_HEIGHT, Robot* _robot, std::vector<Drawable*>& _drawables) : SCREEN_WIDTH(_SCREEN_WIDTH), SCREEN_HEIGHT(_SCREEN_HEIGHT), robot(_robot), drawables(_drawables) {}

  bool init() {
    //Initialization flag
    bool success = true;

    //Initialize SDL
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
      printf("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
      success = false;
    }
    else {
      // TTF_Init();
      //Use OpenGL 2.1
      SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
      SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

      //Create window
      gWindow = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
      if (gWindow == NULL) {
        printf("Window could not be created! SDL Error: %s\n", SDL_GetError());
        success = false;
      }
      else {
        //Create context
        gContext = SDL_GL_CreateContext(gWindow);


        // // renderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED);
        // if (!CreateRenderer()) {
        //   std::cout << "Error in CreateRenderer()" << std::endl;
        //   return false;
        // }
        // SetupRenderer();

        if (gContext == NULL) {
          printf("OpenGL context could not be created! SDL Error: %s\n", SDL_GetError());
          success = false;
        }
        else {
          //Use Vsync
          if (SDL_GL_SetSwapInterval(1) < 0) {
            printf("Warning: Unable to set VSync! SDL Error: %s\n", SDL_GetError());
          }

          //Initialize OpenGL
          if (!initGL()) {
            printf("Unable to initialize OpenGL!\n");
            success = false;
          }
          else {
            // if (!SetupTTF("Georgia.ttf")) {
            //   printf("Warning: Unable to Setup TTF! SDL Error: %s\n", SDL_GetError());
            //   return false;
            // }
            // CreateTextTextures();
          }
        }
      }
    }

    return success;
  }

  void SDLMainLoop() {
    bool quit = false;
    SDL_Event e;

    SDL_StartTextInput();

    while (!quit) {
      while (SDL_PollEvent(&e) != 0) {
        if (e.type == SDL_QUIT) {
          quit = true;
        }
        else if (e.type == SDL_TEXTINPUT) {
          int x = 0, y = 0;
          SDL_GetMouseState(&x, &y);
          handleKeys(e.text.text[ 0 ], x, y);
        }
        else if (e.type == SDL_KEYDOWN) {
          handleKeys(e.key.keysym.sym);
        }
      }

      if (simulate)
        robot->simulateOneSecond();

      if (robot->isFinished())
        simulate = false;

      render(); //Render quad

      SDL_GL_SwapWindow(gWindow); //Update screen
    }

    SDL_StopTextInput(); //Disable text input
  }

  bool initGL() {
    bool success = true;
    GLenum error = GL_NO_ERROR;

    //Initialize Projection Matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0f, (GLfloat)SCREEN_WIDTH / (GLfloat)SCREEN_HEIGHT, 0.1f, 100.0f);

    //Check for error
    error = glGetError();
    if (error != GL_NO_ERROR) {
      printf("Error initializing OpenGL! %s\n", gluErrorString(error));
      success = false;
    }

    //Initialize Modelview Matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // glTranslatef(0.0f, 0.0f, 0.5f);

    //Check for error
    error = glGetError();
    if (error != GL_NO_ERROR) {
      printf("Error initializing OpenGL! %s\n", gluErrorString(error));
      success = false;
    }

    //Initialize clear color
    glClearColor(0.f, 0.f, 0.f, 1.f);

    //Check for error
    error = glGetError();
    if (error != GL_NO_ERROR) {
      printf("Error initializing OpenGL! %s\n", gluErrorString(error));
      success = false;
    }

    return success;
  }

  void render() {
    // // Clear the window and make it all red
    // SDL_RenderClear(renderer);

    // // Render our text objects ( like normal )
    // SDL_RenderCopy(renderer, solidTexture, nullptr, &solidRect);
    // SDL_RenderCopy(renderer, blendedTexture, nullptr, &blendedRect);
    // SDL_RenderCopy(renderer, shadedTexture, nullptr, &shadedRect);

    // // Render the changes above
    // SDL_RenderPresent(renderer);
    // return;
    //Clear color buffer
    glClear(GL_COLOR_BUFFER_BIT);

    glLoadIdentity();                 // Reset the model-view matrix
    glTranslatef(-2.5f, -3.5f, -8.5f);  // Move right and into the screen

    glPushMatrix();
    if (zoom) {
      glTranslatef(-4.0f, 6.0f, -4.0f);
      glRotatef(270.0f, 0.0f, 0.0f, 1.0f);
    }


    /*glColor3f(1.0,0.0,1.0);
    for (size_t i = 0; i < robot->pointsInPolygon.size(); i++) {
      float rad = 0.06;
      glBegin( GL_LINES );
        glVertex2f( robot->pointsInPolygon[i].x-rad, robot->pointsInPolygon[i].y-rad );
        glVertex2f( robot->pointsInPolygon[i].x+rad, robot->pointsInPolygon[i].y+rad );
        glVertex2f( robot->pointsInPolygon[i].x+rad, robot->pointsInPolygon[i].y-rad );
        glVertex2f( robot->pointsInPolygon[i].x-rad, robot->pointsInPolygon[i].y+rad );
      glEnd();
    }*/

    for (size_t i = 0; i < drawables.size(); i++) {
      drawables[i]->draw();
    }

    glPopMatrix();

    if (zoom)
      glTranslatef(-1.5f, 0, 0);
    float height = 6;
    float height_offset = 0.01;
    float y = -0.9;

    GLUtils::drawLine(GLUtils::White, Vec(y, height_offset), Vec(y, height_offset + (float)robot->getCollisions() / 100.0f * height));

    Robot::PassedTime passedTime = robot->time_passed();
    y -= 0.2;
    GLUtils::drawLine(GLUtils::White, Vec(y, height_offset), Vec(y, height_offset + (float)passedTime.seconds / 100.0f * height));
    y -= 0.1;
    GLUtils::drawLine(GLUtils::White, Vec(y, height_offset), Vec(y, height_offset + (float)passedTime.minutes / 100.0f * height));
    y -= 0.1;
    GLUtils::drawLine(GLUtils::White, Vec(y, height_offset), Vec(y, height_offset + (float)passedTime.hours / 100.0f * height));

    y -= 0.2;
    for (int i = 0; i < 100; i++) {
      GLUtils::Color color = GLUtils::White;
      if (i % 60 == 0)
        color = GLUtils::Yellow;
      else if (i % 10 == 0)
        color = GLUtils::Red;
      else if (i % 5 == 0)
        color = GLUtils::Green;
      GLUtils::drawLine(color, Vec(y - 0.05f, height_offset + (float)i / 100.0f * height), Vec(y, (float)i / 100.0f * height));
    }


    // TTF_Font* Sans = TTF_OpenFont("Sans.ttf", 24); //this opens a font style and sets a size

    // SDL_Color White = {255, 255, 255};  // this is the color in rgb format, maxing out all would give you the color white, and it will be your text's color

    // SDL_Surface* surfaceMessage = TTF_RenderText_Solid(Sans, "put your text here", White); // as TTF_RenderText_Solid could only be used on SDL_Surface then you have to create the surface first

    // SDL_Texture* Message = SDL_CreateTextureFromSurface(renderer, surfaceMessage); //now you can convert it into a texture

    // SDL_Rect Message_rect; //create a rect
    // Message_rect.x = 0;  //controls the rect's x coordinate
    // Message_rect.y = 0; // controls the rect's y coordinte
    // Message_rect.w = 100; // controls the width of the rect
    // Message_rect.h = 100; // controls the height of the rect

    // //Mind you that (0,0) is on the top left of the window/screen, think a rect as the text's box, that way it would be very simple to understance

    // //Now since it's a texture, you have to put RenderCopy in your game loop area, the area where the whole code executes

    // SDL_RenderCopy(renderer, Message, NULL, &Message_rect);
  }

  void close() {
    //Destroy window
    SDL_DestroyWindow(gWindow);
    gWindow = NULL;

    // TTF_CloseFont(font);

    //Quit SDL subsystems
    SDL_Quit();
  }

  void handleKeys(unsigned char key, int x, int y) {
    //Toggle quad
    if (key == 'q') {
      SDL_Event quit_event;
      quit_event.type = SDL_QUIT;
      SDL_PushEvent(&quit_event);
    }

    if (key == 'm') {
      simulate = !simulate;
    }

    if (key == 'w') {
      robot->simulateOneSecond();
      // robot->simulateOneTenthSecond();
      // std::cout << "....." << std::endl;
    }

    if (key == 't') {
      Vec selectedPos = GLUtils::getSelectedPos(x, y);
      robot->setPosition(selectedPos);
    }

    if (key == 'a') {
      robot->rotate(-0.1);
    }

    if (key == 'd') {
      robot->rotate(0.1);
    }

    if (key == 'z') {
      zoom = !zoom;
    }

    // if (key == 's') {
    //   robot->x = 2.9;
    //   robot->y = 0.2;
    //   robot->alpha = M_PI;
    //   robot->move();
    //   robot->alpha = M_PI;
    // }

    if (key == 't') {
      // robot->x = 0.8;
      // robot->y = 4.9;
      // robot->alpha = 0;
      // robot->simulateOneSecond();
      // robot->x = 1.0;
      // robot->y = 4.9;
      // robot->alpha = 0;
      // robot->simulateOneSecond();
      // robot->alpha = 0;
    }

    if (key == 'u') {
      unsigned int max = 4000;
      for (size_t i = 0; i < max; i++) {
        if (i % 100 == 0)
          std::cout << i << "/" << max << std::endl;
        robot->simulateOneSecond();
      }
    }

    if (key == 'h') {
      // std::vector<int> path;
      // Eigen::VectorXd min_distance(4);
      // Eigen::VectorXi previous(4);

      // int source = 0;//, min_distance, previous;
      // std::set<int> targets = {2,3};
      // std::vector<std::vector<int> > VV = {{1,3}, {0,2}, {1,3}, {2,0}};

      // Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> > mat1;
      // Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 1, 0, -1, 1> > mat2;
      // // igl::dijkstra_compute_paths<int, Eigen::Matrix<double, -1, 1, 0, -1, 1>, Eigen::Matrix<int, -1, 1, 0, -1, 1> >(source, targets, VV, mat1, mat2);

      // int vertex_found = igl::dijkstra_compute_paths(source, targets, VV, min_distance, previous);
      // std::cout << vertex_found << std::endl;

      // std::cout << std::endl;
      // std::cout << min_distance << std::endl;
      // std::cout << std::endl;
      // std::cout << previous << std::endl;
      // std::cout << std::endl;

      // if (vertex_found != -1) {
      //   std::vector<int> path;
      //   igl::dijkstra_get_shortest_path_to(vertex_found, previous, path);
      //   for (size_t i = 0; i < path.size()-1; i++) {
      //     std::cout << path[i] << std::endl;
      //   }
      // }
    }

    robot->handleKeys(key, x, y);
  }

  void handleKeys(SDL_Keycode key) {
    // switch (key) {
    //   case SDLK_UP:
    //     robot->selectedPolygon++;
    //     break;
    //   case SDLK_DOWN:
    //     robot->selectedPolygon--;
    //     break;
    // }
  }

  bool SetupTTF(const std::string& fontName) {
    // SDL2_TTF needs to be initialized just like SDL2
    if (TTF_Init() == -1) {
      std::cout << " Failed to initialize TTF : " << SDL_GetError() << std::endl;
      return false;
    }

    // Load our fonts, with a huge size
    font = TTF_OpenFont(fontName.c_str(), 90);

    // Error check
    if (font == nullptr) {
      std::cout << " Failed to load font : " << SDL_GetError() << std::endl;
      return false;
    }

    return true;
  }
  void CreateTextTextures() {
    SDL_Surface* solid = TTF_RenderText_Solid(font, "solid", textColor);
    solidTexture = SurfaceToTexture(solid);

    SDL_QueryTexture(solidTexture, NULL, NULL, &solidRect.w, &solidRect.h);
    solidRect.x = 0;
    solidRect.y = 0;

    SDL_Surface* blended = TTF_RenderText_Blended(font, "blended", textColor);
    blendedTexture = SurfaceToTexture(blended);

    SDL_QueryTexture(blendedTexture, NULL, NULL, &blendedRect.w, &blendedRect.h);
    blendedRect.x = 0;
    blendedRect.y = solidRect.y + solidRect.h +  20;

    SDL_Surface* shaded = TTF_RenderText_Shaded(font, "shaded", textColor, backgroundColor);
    shadedTexture = SurfaceToTexture(shaded);

    SDL_QueryTexture(shadedTexture , NULL, NULL, &shadedRect.w, &shadedRect.h);
    shadedRect.x = 0;
    shadedRect.y = blendedRect.y + blendedRect.h + 20;
    std::cout << "Finished CreateTextTextures()" << std::endl;
  }
  // Convert an SDL_Surface to SDL_Texture. We've done this before, so I'll keep it short
  SDL_Texture* SurfaceToTexture(SDL_Surface* surf) {
    SDL_Texture* text;

    text = SDL_CreateTextureFromSurface(renderer, surf);

    SDL_FreeSurface(surf);

    return text;
  }
  bool CreateRenderer() {
    renderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED);

    if (renderer == nullptr) {
      std::cout << "Failed to create renderer : " << SDL_GetError();
      return false;
    }

    return true;
  }
  void SetupRenderer() {
    // Set size of renderer to the same as window
    SDL_RenderSetLogicalSize(renderer, windowRect.w, windowRect.h);

    // Set color of renderer to red
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
  }
};
