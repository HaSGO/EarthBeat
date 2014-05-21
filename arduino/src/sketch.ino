/* vim: ft=arduino */
/* EarthBeat Arduino code
 * Copyright (C) 2014 HackerSpace GO
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

int x, y, z;

byte b, c, d, e, f;

byte serin;

byte a = 0;
byte g = 0;

int cont = 0;

void setup() {
    Serial.begin(115200);
}

void loop() {


    if (Serial.available() > 0) {
        serin = Serial.read();
        if (serin == 'c') {
            cont = 1;
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);
        }
        if (serin == 's') {
            cont = 0;
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);
            Serial.write(0);

        }
    }

    if (serin == 'r' || cont == 1) {
        serin = '0';
        b = 0;
        c = 0;
        d = 0;
        e = 0;
        f = 0;

        x = analogRead(A0);
        y = analogRead(A1);
        z = analogRead(A2);

        b = b | x >> 7;
        c = c | x << 1;
        c = c | y >> 9;
        d = d | y >> 1;
        e = e | y << 7;
        e = e | z >> 3;
        f = f | z << 5;

        Serial.write(a);
        Serial.write(b);
        Serial.write(c);
        Serial.write(d);
        Serial.write(e);
        Serial.write(f);
        Serial.write(g);

        delay(2);
    }
}
