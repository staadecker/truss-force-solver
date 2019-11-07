# 2D Truss Force Solver

This program calculates the forces present in the truss members of a bridge as shown in the pictures below.

## Running the code

1. Download the code on your laptop.
2. Make sure you have Python 3, and the Python Quantities library installed ([here](https://pythonhosted.org/quantities/))
3. Follow the instructions in `main.py` to input your bridge design.
3. Run `main.py`!

## Supported Functionality

- Calculation of forces in Pratt, Howe, Warren and K trusses
- Point loads and uniformly distributed loads
- Calculating minimum second moment of area to avoid buckling
- Calculating minimum cross-sectional area to avoid yielding

## Limitations

- All loads and supports have to be in one horizontal axis (i.e. the deck).
Point loads must be pointing downwards. Removing this limitation would be the next logical improvement.
 
## Known Issues

- No unit testing. This is a personal project that I developed for fun rather than reliability.
Therefore, I decided to not focus on testing. This was a intentional decision.
Do not expect everything to work perfectly, especially with weird loading cases.

## License

This project is licenced under the MIT License. Make sure to cite properly and see `LISENCE` for more details.