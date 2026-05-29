import os

filepath = r'C:\Users\rishi\.gemini\antigravity\scratch\drone_project\drone_combat_sim.m'
with open(filepath, 'r', encoding='utf-8') as f:
    code = f.read()

# 1. Remove Decoys
code = code.replace("decoys = [30 110; 110 30; 90 90];", "decoys = [];")
code = code.replace("decoys = [22 72; 78 18; 58 52];", "decoys = [];")

# 2. Fix graphics (Friendly)
square_f = """        % Drone body - square (same size and shape as enemy, but blue)
        sq = 3.5;
        fill([fs(1)-sq fs(1)+sq fs(1)+sq fs(1)-sq], ...
             [fs(2)-sq fs(2)-sq fs(2)+sq fs(2)+sq], ...
             fc, 'EdgeColor', 'w', 'LineWidth', 1.8);

        % Heading arrow
        quiver(fs(1), fs(2), 6*cos(fs(3)), 6*sin(fs(3)), 0, ...
               'Color', 'w', 'LineWidth', 2.5, 'MaxHeadSize', 2.0);"""

circle_f = """        % Drone body - circle
        tc = linspace(0, 2*pi, 20);
        fill(fs(1)+3.5*cos(tc), fs(2)+3.5*sin(tc), ...
             fc, 'EdgeColor', 'w', 'LineWidth', 1.8);

        % Heading arrow (start from edge of circle)
        plot([fs(1)+3.5*cos(fs(3)), fs(1)+7.5*cos(fs(3))], ...
             [fs(2)+3.5*sin(fs(3)), fs(2)+7.5*sin(fs(3))], 'w-', 'LineWidth', 2.5);"""
code = code.replace(square_f, circle_f)

# 3. Fix graphics (Enemy)
# Need to find enemy rendering code. Let's look at it.
square_e = """        % Drone body - square
        sq = 3.5;
        fill([es(1)-sq es(1)+sq es(1)+sq es(1)-sq], ...
             [es(2)-sq es(2)-sq es(2)+sq es(2)+sq], ...
             ec, 'EdgeColor', 'k', 'LineWidth', 1.8);

        % Heading arrow
        quiver(es(1), es(2), 6*cos(es(3)), 6*sin(es(3)), 0, ...
               'Color', 'k', 'LineWidth', 2.5, 'MaxHeadSize', 2.0);"""

circle_e = """        % Drone body - circle
        tc = linspace(0, 2*pi, 20);
        fill(es(1)+3.5*cos(tc), es(2)+3.5*sin(tc), ...
             ec, 'EdgeColor', 'k', 'LineWidth', 1.8);

        % Heading arrow
        plot([es(1)+3.5*cos(es(3)), es(1)+7.5*cos(es(3))], ...
             [es(2)+3.5*sin(es(3)), es(2)+7.5*sin(es(3))], 'k-', 'LineWidth', 2.5);"""

# The enemy color logic might wrap this, so just replacing the shape is better.
# Wait, let's write a regex to replace the square fill logic for enemies.
import re
code = re.sub(
    r'sq = 3\.5;\n\s*fill\(\[es\(1\)-sq es\(1\)\+sq es\(1\)\+sq es\(1\)-sq\], \.\.\.\n\s*\[es\(2\)-sq es\(2\)-sq es\(2\)\+sq es\(2\)\+sq\], \.\.\.\n\s*ec, \'EdgeColor\', \'w\', \'LineWidth\', 1\.8\);',
    r'tc = linspace(0, 2*pi, 20);\n        fill(es(1)+3.5*cos(tc), es(2)+3.5*sin(tc), ec, \'EdgeColor\', \'w\', \'LineWidth\', 1.8);',
    code
)
# And the quiver
code = re.sub(
    r'quiver\(es\(1\), es\(2\), 6\*cos\(es\(3\)\), 6\*sin\(es\(3\)\), 0, \.\.\.\n\s*\'Color\', \'w\', \'LineWidth\', 2\.5, \'MaxHeadSize\', 2\.0\);',
    r'plot([es(1)+3.5*cos(es(3)), es(1)+7.5*cos(es(3))], [es(2)+3.5*sin(es(3)), es(2)+7.5*sin(es(3))], \'w-\', \'LineWidth\', 2.5);',
    code
)
# I will just write a python script to fetch the enemy drawing lines and replace them.

with open(filepath, 'w', encoding='utf-8') as f:
    f.write(code)

print("done")
