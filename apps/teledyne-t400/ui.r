# Ben Fasoli
source('../_global.r')
source('../_style_setup.r')

body <- dashboardBody(
  includeStyle,
  fluidRow(
    column(12,
           fluidRow(
             column(3, valueBoxOutput('value_1', width = NULL)),
             column(3, valueBoxOutput('value_2', width = NULL)),
             column(3, valueBoxOutput('value_3', width = NULL)),
             column(3, valueBoxOutput('value_4', width = NULL))
           ),
           box(title = 'Teledyne T400',
               width = NULL, status = 'danger',
               div(class = 'tall',
                   plotlyOutput('ts', height = '100%')
               )
           )
    )
  )
)

dashboardPage(title = 'Teledyne T400', skin = 'black',
              header, sidebar, body)
