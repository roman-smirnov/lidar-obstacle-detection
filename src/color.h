//
// Created by Roman Smirnov on 2019-06-19.
//

#ifndef COLOR_H_
#define COLOR_H_

struct Color final {
  const float r, g, b;

  constexpr explicit Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}

  static constexpr Color getRed() noexcept {
    return Color(1.0, 0.0, 0.0);
  }

  static constexpr Color getGreen() noexcept {
    return Color(0.0, 1.0, 0.0);
  }

  static constexpr Color getBlue() noexcept {
    return Color(0.0, 0.0, 1.0);
  }

  static constexpr Color getYellow() noexcept {
    return Color(1.0, 1.0, 0.0);
  }

  static constexpr Color getCyan() noexcept {
    return Color(0.0, 1.0, 1.0);
  }

  static constexpr Color getPurple() noexcept {
    return Color(1.0, 0.0, 1.0);
  }

  static constexpr Color getLightGray() noexcept {
    return Color(0.8, 0.8, 0.8);
  }

  static constexpr Color getMedGray() noexcept {
    return Color(0.5, 0.5, 0.5);
  }

  static constexpr Color getDarkGray() noexcept {
    return Color(0.3, 0.3, 0.3);
  }
};

#endif //COLOR_H_
