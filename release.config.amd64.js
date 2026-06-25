
const ROS_DISTRO = process.env.ROS_DISTRO || 'lyrical';
// Deepstream only supports Ubuntu 24.04, so skip GPU images for distros without it.
const DISTROS_WITHOUT_GPU = ['lyrical'];
const NO_GPU = DISTROS_WITHOUT_GPU.includes(ROS_DISTRO) ? ' --no-gpu' : '';

module.exports = {
  branches: ['3.x', 'main'],
  plugins:
    [
      [
        '@semantic-release/commit-analyzer',
        { 'preset': 'conventionalcommits' }
      ],
      [
        '@semantic-release/release-notes-generator',
        { 'preset': 'conventionalcommits' }
      ],
      [
        "@semantic-release/exec",
        {
          "prepareCmd": `scripts/docker-build.py --version \${nextRelease.version} --arch amd64 --ros_distro ${ROS_DISTRO}${NO_GPU}`,
          "publishCmd": `scripts/docker-build.py --version \${nextRelease.version} --arch amd64 --ros_distro ${ROS_DISTRO} --push${NO_GPU}`
        }
      ],
      [
        "@semantic-release/exec",
        {
          "publishCmd": 'echo "published=true" >> "$GITHUB_OUTPUT" && echo "version=${nextRelease.version}" >> "$GITHUB_OUTPUT"'
        }
      ],
      [
        "@semantic-release/github",
        {
          "successComment": false,
          "failComment": false
        }
      ]
    ],
};
