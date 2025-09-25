import numpy as np
import matplotlib.pyplot as plt

class PipePlot:
    def __init__(self, outer_radius, inner_radius, length, highlight_distance, highlight_width):
        self.outer_radius = outer_radius
        self.inner_radius = inner_radius
        self.length = length
        self.highlight_distance = highlight_distance
        self.highlight_width = highlight_width
        self.blue = np.array([0, 0, 1, 1])
        self.red = np.array([1, 0, 0, 1])

    def hotspot(self, z):
        return np.exp(-((z - self.highlight_distance) ** 2) / (2 * self.highlight_width ** 2))

    def mix_color(self, z_array):
        w = self.hotspot(z_array)
        c = (1 - w)[..., None] * self.blue + w[..., None] * self.red
        return c

    def plot(self):
        theta = np.linspace(0, 2 * np.pi, 200)
        z = np.linspace(0, self.length, 200)
        Theta, Z = np.meshgrid(theta, z)

        Xo = self.outer_radius * np.cos(Theta)
        Yo = self.outer_radius * np.sin(Theta)
        Zo = Z

        R = np.linspace(0, self.outer_radius, 200)
        Theta_cap, R_cap = np.meshgrid(theta, R)
        Xcap = R_cap * np.cos(Theta_cap)
        Ycap = R_cap * np.sin(Theta_cap)
        Zcap0 = np.zeros_like(Xcap)
        ZcapL = np.full_like(Xcap, self.length)

        colors_outer = self.mix_color(Zo)
        colors_cap0 = self.mix_color(Zcap0)
        colors_capL = self.mix_color(ZcapL)

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(projection='3d')

        ax.plot_surface(Xo, Yo, Zo, facecolors=colors_outer, linewidth=0, antialiased=False, shade=False)
        ax.plot_surface(Xcap, Ycap, Zcap0, facecolors=colors_cap0, linewidth=0, antialiased=False, shade=False)
        ax.plot_surface(Xcap, Ycap, ZcapL, facecolors=colors_capL, linewidth=0, antialiased=False, shade=False)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Solid Pipe with Local Red Gradient')

        max_range = max(Xo.max()-Xo.min(), Yo.max()-Yo.min(), self.length)
        ax.set_xlim(-max_range/2, max_range/2)
        ax.set_ylim(-max_range/2, max_range/2)
        ax.set_zlim(0, self.length)

        plt.tight_layout()
        plt.show()


pipe = PipePlot(outer_radius=0.1524/2, inner_radius=0.14/2, length=1.0, highlight_distance=1-0.362, highlight_width=0.1)
pipe.plot()
