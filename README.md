# 🔌 Arduino

<div align="center">

<img src="https://img.shields.io/badge/arduino-latest-00979D.svg?style=for-the-badge&logo=arduino" alt="Arduino">
<img src="https://img.shields.io/badge/c++-latest-00599C.svg?style=for-the-badge&logo=cplusplus" alt="C++">
<img src="https://img.shields.io/badge/license-MIT-green.svg?style=for-the-badge" alt="License">

**Arduino hardware experiments and sketches**

</div>

---

A collection of Arduino sketches and hardware experiments initialized with `init-arduino.sh`.

---

## Usage

Open any `.ino` sketch in the [Arduino IDE](https://www.arduino.cc/en/software) or compile with the Arduino CLI:

```bash
arduino-cli compile --fqbn arduino:avr:uno <sketch>
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno <sketch>
```

---

## License

MIT — see [LICENSE](LICENSE).

---

<div align="center">
Made with ❤️ by <a href="https://github.com/NoamFav">NoamFav</a>
</div>
