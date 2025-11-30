/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */

const sidebars = {

  tutorialSidebar: [

    {

      type: 'category',

      label: 'Part 1: Foundations',

      collapsible: true,

      collapsed: false,

      items: [

        'chapter-01',

        'chapter-02',

        'chapter-03',

      ],

    },

    {

      type: 'category',

      label: 'Part 2: Perception & Control',

      collapsible: true,

      collapsed: false,

      items: [

        'chapter-04',

        'chapter-05',

        'chapter-06',

      ],

    },

    {

      type: 'category',

      label: 'Part 3: Advanced Robotics',

      collapsible: true,

      collapsed: false,

      items: [

        'chapter-07',

        'chapter-08',

        'chapter-09',

        'chapter-10',

      ],

    },

  ],

};



module.exports = sidebars;

